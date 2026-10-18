"""
ONLINE perturbation detector based on the frozen forward model (FM).

At every sample (10 Hz) the FM predicts sens(t) in two ways:
  mode "1" (one-step) : TRUE sensor history       t-H .. t-1
  mode "R" (rollout)  : PREDICTED sensor history  (the FM does not see the real
                        sensors again until the rollout is re-anchored)
For each mode m and channel c:
  residual  e = y - y_pred
  score     S(t) = sqrt( 1/W * sum_{k<W} (e(t-k) / sigma)^2 )
sigma and thresholds come from the calibration (calibrate.py) and are stored
in a JSON file.

State machine:
  STARTUP     fills the history with H real samples
  NORMAL      computes the scores; re-anchors the rollout to the real data if
              everything has been quiet for quiet_s and the last re-anchor is
              older than min_reanchor_s
  ALARM       S_R of a channel crossed its threshold at t_a; after T seconds
              each channel is classified:
                persistent   S_R above threshold in the last Tc seconds,
                             or the channel's CUSUM above h and still rising
                             on average over the last Tc
                             (persist_cusum: persistent perturbations close
                             to the threshold, where S_R oscillates)
                transient    S_R above threshold only earlier
                none         channel never above threshold
              and the onset of each channel is labelled from how fast S_R
              rose: "abrupt" if, in the Tb before the crossing, S_R grew by
              more than rise_frac * threshold, otherwise "gradual";
              "undetermined" if the alarm fires right after startup (less
              than Tb of valid S_R). S_1 is computed and logged but does not
              decide the label: it only enters the quiet condition of the
              re-anchor.
  PERSISTENT  keeps computing the scores; when S_R of every channel stays
              below threshold for quiet_s it returns to NORMAL on its own and
              records end time and duration in the event. reset() is still
              available to force the exit manually.

SLOW detector (CUSUM), in parallel with the state machine, for small but
persistent perturbations (e.g. missing fin) that never cross the thresholds.
Default (chosen from the data): ONE-STEP residual, FIXED reference, kappa = 1.
A 10 s session reference proved unreliable: the model error varies 10-18x
between 3 s blocks even in normal conditions.
  z2(t)  = min( (e(t)/sigma_C)^2 , cusum_clip )    cusum_mode residual, per channel
  cusum_mode "1i" (default): one-step, ISOLATED on the channels in cusum_iso.
           For an isolated channel its own history is real and the other
           channel's history comes from the rollout (which does not see
           perturbations): a perturbation on the other channel does not leak
           into it. Default: only sensor_diff isolated (a perturbed current
           leaked into sensor_diff; isolating current too makes it sensitive
           to the sensor_diff rollout errors and raises its threshold a lot).
           sigma_C = cusum_sigma.
  ref    = mean of z2 over the first cusum_ref_s seconds in NORMAL
           (reference of the robot in THIS session; frozen afterwards)
           cusum_ref_s = 0 -> ref = 1 (fixed reference of the normal trials)
  g(t)   = max(0, g(t-1) + z2/ref - 1 - cusum_kappa)
  slow alarm on the channel if g > h (h = cusum_h, from calibrate.py);
  it ends when g drops below cusum_off * h and stays there for cusum_merge_s
  (if g exceeds h again in the meantime the same event continues); the
  recorded end is the instant of the drop. g is capped at 1.2 h.
  If a slow alarm starts while the other channel is already in alarm, it is
  marked "secondary" with the channel of origin: in the synthetic tests the
  perturbed channel always alarms first.
  The two detectors stay SEPARATE: the fast one classifies large deviations
  (transient/persistent), the slow one reports a small persistent change in
  progress. Note: the slow one uses the one-step residual, so a perturbation
  on one channel can also trigger the other (the perturbed history affects
  both predictions): the channel is indicative. It does not change the state
  of the state machine; it only takes part in the persistent/transient
  classification of a channel that is already in fast alarm (persist_cusum).
  NB: with a session reference it detects changes WITH RESPECT TO THE START
  of the session: a perturbation already present at power-on becomes part of
  the reference. reset_reference() starts again with a new reference.

All 2x2 arrays are indexed [mode, channel] with MODES and CHANNELS.
This file only depends on numpy and torch: it can be ported to the robot.
"""
import json
import warnings
from collections import deque
from dataclasses import dataclass, asdict, field

import numpy as np
import torch

CHANNELS = ("sensor_diff", "current")
MODES    = ("1", "R")
M1, MR   = 0, 1
SLOW_MODES = ("1i", "1", "R")    # possible residuals for the slow detector

STARTUP, NORMAL, ALARM, PERSISTENT = "startup", "normal", "alarm", "persistent"


@dataclass
class DetectorConfig:
    hz:             float = 10.0   # sampling frequency
    W_s:            float = 0.5    # score window
    T_s:            float = 6.5    # wait between alarm and classification (transient if < T - Tc - W = 5 s)
    Tc_s:           float = 1.0    # last part of T: is S_R still above threshold?
    Tb_s:           float = 0.7    # window over which the rise of S_R is measured
    rise_frac:      float = 0.5    # minimum rise (fraction of threshold) for "abrupt"
    quiet_s:        float = 5.0    # quiet time required before a re-anchor
    min_reanchor_s: float = 20.0   # minimum interval between re-anchors
    # --- slow detector (CUSUM) ---
    cusum_mode:     str   = "1i"   # residual: "1i" one-step, isolated on the cusum_iso channels
                                   #           (default), "1" one-step, "R" rollout
    cusum_iso:      str   = "sensor_diff"  # channels isolated in mode "1i" (comma separated)
    cusum_ref_s:    float = 0.0    # 0 = fixed reference (default); >0 = session reference
    cusum_kappa:    float = 1.0    # g grows if z2 (clipped) exceeds 1 + kappa on average
    cusum_clip:     float = 9.0    # limit on the z2 of a single sample (3 sigma)
    cusum_off:      float = 0.5    # the slow alarm ends when g < cusum_off * h ...
    cusum_merge_s:  float = 5.0    # ... and stays there for cusum_merge_s (otherwise it continues)
    persist_cusum:  bool  = True   # at classification: persistent also if g > h and rising
                                   # (only channels already in fast alarm)

    def n(self, seconds):
        """Seconds -> number of samples (at least 1)."""
        return max(1, int(round(seconds * self.hz)))


def cusum_run(z2, ref_n, kappa, clip):
    """Offline CUSUM on a z2 series (n, 2), same maths as the online detector
    (without the cap). Reference = mean of the first ref_n samples
    (ref_n = 0 -> reference 1). Returns (g (n,2) with NaN during the
    reference, ref (2,))."""
    z2 = np.minimum(np.asarray(z2, dtype=np.float64), clip)
    ref = np.maximum(z2[:ref_n].mean(axis=0), 1e-3) if ref_n > 0 else np.ones(z2.shape[1])
    g = np.full_like(z2, np.nan)
    acc = np.zeros(z2.shape[1])
    for t in range(ref_n, len(z2)):
        acc = np.maximum(0.0, acc + z2[t] / ref - 1.0 - kappa)
        g[t] = acc
    return g, ref


@dataclass
class Calibration:
    sigma:     np.ndarray                      # (2,2) [mode, channel]
    threshold: np.ndarray                      # (2,2) [mode, channel]
    config:    DetectorConfig = field(default_factory=DetectorConfig)
    meta:      dict = field(default_factory=dict)
    cusum_h:   np.ndarray = field(default_factory=lambda: np.full(2, np.inf))  # (2,) channel
    cusum_sigma: np.ndarray = None    # (2,) normaliser of the slow residual (None = sigma of the mode)

    @staticmethod
    def neutral(config=None):
        """sigma=1 and infinite thresholds: the detector never alarms and the
        score is the raw RMS of the residual. Used by calibrate.py."""
        return Calibration(np.ones((2, 2)), np.full((2, 2), np.inf),
                           config or DetectorConfig(), cusum_h=np.full(2, np.inf))

    @staticmethod
    def _to_named(a):
        return {m: {c: float(a[mi, ci]) for ci, c in enumerate(CHANNELS)}
                for mi, m in enumerate(MODES)}

    @staticmethod
    def _from_named(d):
        return np.array([[float(d[m][c]) for c in CHANNELS] for m in MODES])

    def save(self, path):
        with open(path, "w") as f:
            json.dump({"sigma": self._to_named(self.sigma),
                       "threshold": self._to_named(self.threshold),
                       "cusum_h": {c: float(self.cusum_h[ci]) for ci, c in enumerate(CHANNELS)},
                       "cusum_sigma": (None if self.cusum_sigma is None else
                                       {c: float(self.cusum_sigma[ci]) for ci, c in enumerate(CHANNELS)}),
                       "config": asdict(self.config),
                       "meta": self.meta}, f, indent=2)

    @staticmethod
    def load(path):
        with open(path) as f:
            d = json.load(f)
        # cusum_h missing (old JSON files) -> slow detector disabled
        ch = d.get("cusum_h", {})
        cusum_h = np.array([float(ch.get(c, np.inf)) for c in CHANNELS])
        cs = d.get("cusum_sigma")
        cusum_sigma = None if cs is None else np.array([float(cs[c]) for c in CHANNELS])
        return Calibration(Calibration._from_named(d["sigma"]),
                           Calibration._from_named(d["threshold"]),
                           DetectorConfig(**d["config"]), d.get("meta", {}), cusum_h,
                           cusum_sigma)


class PerturbationDetector:
    """Usage:
        det = PerturbationDetector(FM, Calibration.load("calib.json"), H=20)
        for cmd, sens in stream:            # NORMALISED values
            out = det.step(cmd, sens)
            if out["alarm"]: ...            # alarm just fired
            if out["event"]: ...            # classification just decided
            if out["cleared"]: ...          # persistent perturbation ended
            if out["slow_alarm"]: ...       # slow detector alarm (channels)
            if out["slow_cleared"]: ...     # end of a slow alarm
    On the robot, with physical values: pass norm_stats (from the checkpoint)
    and use step_raw(cmd_rad, sensor_diff, current_ma).
    """

    def __init__(self, FM, calib, H, device="cpu", norm_stats=None):
        self.FM = FM.to(device).eval()
        self.calib = calib
        self.cfg = calib.config
        self.H = int(H)
        self.device = torch.device(device)
        self.norm_stats = norm_stats

        c = self.cfg
        self.nW, self.nT   = c.n(c.W_s), c.n(c.T_s)
        self.nTc, self.nTb = c.n(c.Tc_s), c.n(c.Tb_s)
        self.nQuiet, self.nMin = c.n(c.quiet_s), c.n(c.min_reanchor_s)
        if self.nTc > self.nT:
            raise ValueError("Tc_s must be <= T_s")
        if c.cusum_mode not in SLOW_MODES:
            raise ValueError(f"cusum_mode must be one of {SLOW_MODES}")
        self.iso = c.cusum_mode == "1i"
        iso_ch = [x.strip() for x in c.cusum_iso.split(",") if x.strip()]
        bad = [x for x in iso_ch if x not in CHANNELS]
        if bad:
            raise ValueError(f"cusum_iso: unknown channels {bad}")
        self.iso_mask = np.array([ch in iso_ch for ch in CHANNELS])
        self.mC = M1 if self.iso else MODES.index(c.cusum_mode)
        self.cusum_sigma = (np.asarray(calib.cusum_sigma, dtype=np.float64)
                            if calib.cusum_sigma is not None else calib.sigma[self.mC])
        self.nRef = c.n(c.cusum_ref_s) if c.cusum_ref_s > 0 else 0
        self.h = np.asarray(calib.cusum_h, dtype=np.float64)
        self.nMerge = c.n(c.cusum_merge_s)
        if self.nQuiet < self.H + self.nW:
            warnings.warn(
                f"quiet_s ({self.nQuiet} samples) < H + W ({self.H + self.nW}): "
                f"at re-anchor the real history may contain non-quiet samples.")
        self.start()

    # ------------------------------------------------------------------ state
    def start(self):
        """Reset everything: call at the start of every trial/session."""
        self.t = 0                       # index of the next sample
        self.state = STARTUP
        self.events = []                 # events already classified
        self.reanchor_times = []         # sample indices of the re-anchors
        self._boot_cmd, self._boot_sens = [], []
        self.hist_cmd = self.hist_real = self.hist_roll = None
        self._z = deque(maxlen=self.nW)          # normalised residuals (2,2)
        self._ratioR = deque(maxlen=self.nTb + 1)  # S_R / threshold, last Tb (2,)
        self._age = 0                    # samples since the last re-anchor
        self._quiet = 0                  # consecutive quiet samples
        self._ev = None                  # event in progress (ALARM state)
        self._persist_ev = None          # persistent event still open
        self.slow_events = []            # slow detector events
        self.reset_reference()

    def reset_reference(self):
        """Start again with a new reference for the slow detector: the next
        cusum_ref_s seconds in NORMAL become the new 'normal'."""
        self._ref_sum = np.zeros(2)
        self._ref_n = 0
        self.ref = None if self.nRef > 0 else np.ones(2)
        self.g = np.zeros(2)
        self._slow_ev = [None, None]
        self._pending_end = [None, None]     # instant at which g dropped below off*h
        self._z2_slow = deque(maxlen=self.nTc)   # z2/ref over the last Tc (persist_cusum)

    def reset(self, reanchor=False):
        """MANUAL reset (optional): forces the return to NORMAL from
        PERSISTENT or ALARM. reanchor=True re-anchors the rollout to the real
        sensors immediately: use only if the perturbation is really over."""
        if self.state == STARTUP:
            return
        self.state, self._ev, self._persist_ev, self._quiet = NORMAL, None, None, 0
        if reanchor:
            self._reanchor()

    def _reanchor(self):
        self.hist_roll = self.hist_real.clone()
        self._age = 0
        self.reanchor_times.append(self.t)

    # ------------------------------------------------------------------- step
    def step_raw(self, cmd_rad, sensor_diff, current_ma):
        """Like step(), but with physical values: normalises with norm_stats."""
        ns = self.norm_stats
        if ns is None:
            raise ValueError("step_raw needs norm_stats (from the checkpoint).")
        return self.step((cmd_rad - ns["cmd_mean"]) / ns["cmd_std"],
                         [(sensor_diff - ns["sd_mean"]) / ns["sd_std"],
                          (current_ma - ns["vf_mean"]) / ns["vf_std"]])

    @torch.no_grad()
    def step(self, cmd, sens):
        """One sample: cmd (float) and sens = [sensor_diff, current], NORMALISED.
        Returns a dict; pred/resid/score are (2,2) [mode, channel], NaN during
        STARTUP (score is also NaN until the W window is full)."""
        cmd_t = torch.tensor([[float(cmd)]], dtype=torch.float32, device=self.device)
        y = torch.as_tensor(np.asarray(sens, dtype=np.float32).reshape(1, 2),
                            device=self.device)
        nan = np.full((2, 2), np.nan)
        out = {"t": self.t, "state": self.state, "pred": nan, "resid": nan,
               "score": nan, "age": 0, "alarm": False, "reanchored": False,
               "event": None, "cleared": None,
               "g": np.full(2, np.nan), "slow": np.zeros(2, dtype=bool),
               "resid_iso": np.full(2, np.nan),
               "slow_alarm": [], "slow_cleared": []}

        # ---- STARTUP: fill the history with H real samples ----
        if self.hist_cmd is None:
            self._boot_cmd.append(cmd_t)
            self._boot_sens.append(y)
            if len(self._boot_cmd) == self.H:
                self.hist_cmd  = torch.cat(self._boot_cmd, dim=0)     # (H,1)
                self.hist_real = torch.cat(self._boot_sens, dim=0)    # (H,2)
                self.hist_roll = self.hist_real.clone()
                self._age = 0
                self.state = NORMAL           # effective from the next sample
            self.t += 1
            return out

        # ---- predictions in a single forward pass: row 0 = one-step,
        #      1 = rollout, (if iso) 2 = isolated sensor_diff, 3 = isolated current ----
        rows = [self.hist_real, self.hist_roll]
        if self.iso:
            rows += [torch.cat([self.hist_real[:, 0:1], self.hist_roll[:, 1:2]], dim=1),
                     torch.cat([self.hist_roll[:, 0:1], self.hist_real[:, 1:2]], dim=1)]
        nb = len(rows)
        seq_cmd  = self.hist_cmd.unsqueeze(0).expand(nb, -1, -1)      # (nb,H,1)
        seq_sens = torch.stack(rows)                                  # (nb,H,2)
        pred_all = self.FM(seq_cmd, seq_sens, cmd_t.expand(nb, -1))   # (nb,2)
        pred = pred_all[:2]
        resid = (y - pred).cpu().numpy().astype(np.float64)           # (2,2)
        if self.iso:
            iso = torch.stack([pred_all[2, 0], pred_all[3, 1]])
            out["resid_iso"] = (y[0] - iso).cpu().numpy().astype(np.float64)

        self._age += 1
        self.hist_cmd  = torch.cat([self.hist_cmd[1:],  cmd_t], dim=0)
        self.hist_real = torch.cat([self.hist_real[1:], y], dim=0)
        self.hist_roll = torch.cat([self.hist_roll[1:], pred[MR:MR + 1]], dim=0)

        # ---- score ----
        self._z.append(resid / self.calib.sigma)
        if len(self._z) == self.nW:
            score = np.sqrt(np.mean(np.stack(self._z) ** 2, axis=0))
        else:
            score = nan
        valid = bool(np.isfinite(score).all())
        over = score > self.calib.threshold          # (2,2); NaN -> False
        over_R = over[MR]
        self._ratioR.append(score[MR] / self.calib.threshold[MR])

        # ---- state machine ----
        if self.state == PERSISTENT:
            # automatic exit: S_R of every channel below threshold for quiet_s
            self._quiet = self._quiet + 1 if (valid and not over_R.any()) else 0
            if self._quiet >= self.nQuiet:
                ev = self._persist_ev
                t_end = self.t - self.nQuiet + 1      # start of the quiet stretch
                ev["t_end_s"] = t_end / self.cfg.hz
                ev["duration_s"] = (t_end - ev["t_alarm"]) / self.cfg.hz
                out["cleared"] = ev
                self.state, self._persist_ev, self._quiet = NORMAL, None, 0
        elif self.state == NORMAL:
            if over_R.any():
                self.state = ALARM
                out["alarm"] = True
                self._quiet = 0
                self._ev = {
                    "t_alarm": self.t,
                    "trigger": [c for ci, c in enumerate(CHANNELS) if over_R[ci]],
                    "ever": np.zeros(2, dtype=bool),
                    "late": np.zeros(2, dtype=bool),
                    "abrupt": np.zeros(2, dtype=bool),
                    "rise": np.full(2, np.nan),
                }
            elif valid and not over.any():
                self._quiet += 1
            else:
                self._quiet = 0

        if self.state == ALARM:
            ev = self._ev
            k = self.t - ev["t_alarm"]
            # onset label: fixed for each channel at the moment its OWN S_R
            # crosses the threshold for the first time
            for ci in np.flatnonzero(over_R & ~ev["ever"]):
                ev["rise"][ci] = self._rise(ci)
                ev["abrupt"][ci] = ev["rise"][ci] > self.cfg.rise_frac
            ev["ever"] |= over_R
            if k > self.nT - self.nTc:
                ev["late"] |= over_R
            if k >= self.nT:
                # Persistent perturbation close to the threshold: S_R oscillates
                # around theta and may be below it in the last Tc. The channel is
                # still persistent if two conditions hold: g > h (accumulated
                # evidence) AND the mean CUSUM increment over the last Tc is
                # positive (accumulation still in progress). The second one
                # matters after a finished transient: if the normal residual is
                # just below 1 + kappa, g decreases very slowly and would stay
                # above h even after the perturbation is over.
                # Only for channels that already crossed the fast threshold.
                if self.cfg.persist_cusum and len(self._z2_slow):
                    drift = np.mean(np.stack(self._z2_slow), axis=0) - 1.0 - self.cfg.cusum_kappa
                    by_cusum = ev["ever"] & ~ev["late"] & (self.g > self.h) & (drift > 0)
                    ev["late_cusum"] = by_cusum
                    ev["late"] |= by_cusum
                out["event"] = self._classify(ev)
                self._ev = None

        # ---- rollout re-anchor: only in NORMAL, quiet and not too often ----
        if (self.state == NORMAL and self._quiet >= self.nQuiet
                and self._age >= self.nMin):
            self._reanchor()
            out["reanchored"] = True

        e_slow = (np.where(self.iso_mask, out["resid_iso"], resid[M1]) if self.iso
                  else resid[self.mC])
        self._slow_step(e_slow, out)

        out.update(state=self.state, pred=pred.cpu().numpy(), resid=resid,
                   score=score, age=self._age)
        self.t += 1
        return out

    def _slow_step(self, e, out):
        """Slow detector (CUSUM) on the residual e (2,). First the optional
        session reference, then accumulation."""
        c = self.cfg
        z2 = np.minimum((e / self.cusum_sigma) ** 2, c.cusum_clip)
        if self.ref is None:
            if self.state == NORMAL:               # reference only in quiet conditions
                self._ref_sum += z2
                self._ref_n += 1
                if self._ref_n >= self.nRef:
                    self.ref = np.maximum(self._ref_sum / self._ref_n, 1e-3)
            return
        # g capped at 1.2 h: after the end of the perturbation the slow alarm
        # closes within a few seconds (g drops by ~1+kappa-z2 per sample), so it
        # does not turn a short, already finished perturbation into "persistent".
        self._z2_slow.append(z2 / self.ref)
        cap = np.where(np.isfinite(self.h), 1.2 * self.h, np.inf)
        self.g = np.minimum(np.maximum(0.0, self.g + z2 / self.ref - 1.0 - c.cusum_kappa), cap)
        hz = c.hz
        for ci, ch in enumerate(CHANNELS):
            ev = self._slow_ev[ci]
            if ev is None and self.g[ci] > self.h[ci]:
                # if the other channel is already in alarm, this one is probably
                # an indirect effect (the perturbed history affects the prediction)
                other = [CHANNELS[k] for k in range(len(CHANNELS))
                         if k != ci and self._slow_ev[k] is not None]
                ev = {"channel": ch, "t_alarm": self.t, "t_alarm_s": self.t / hz,
                      "secondary": bool(other), "origin": other[0] if other else ch,
                      "ref": float(self.ref[ci])}
                self._slow_ev[ci] = ev
                self.slow_events.append(ev)
                out["slow_alarm"].append(ch)
            elif ev is not None:
                pend = self._pending_end[ci]
                if self.g[ci] < c.cusum_off * self.h[ci]:
                    if pend is None:
                        self._pending_end[ci] = self.t
                    elif self.t - pend >= self.nMerge:      # below long enough: end
                        ev["t_end_s"] = pend / hz
                        ev["duration_s"] = (pend - ev["t_alarm"]) / hz
                        self._slow_ev[ci] = None
                        self._pending_end[ci] = None
                        out["slow_cleared"].append(ch)
                elif pend is not None and self.g[ci] > self.h[ci]:
                    self._pending_end[ci] = None            # resumed: same event
        out["g"] = self.g.copy()
        out["slow"] = np.array([e is not None for e in self._slow_ev])

    def _slow_active(self):
        """(2,) bool: slow alarm active per channel."""
        return np.array([e is not None for e in self._slow_ev])

    def _rise(self, ci):
        """Growth of S_R/threshold of channel ci over the last Tb seconds
        (current value minus the value Tb ago). NaN if there are not yet Tb
        seconds of valid S_R (alarm right after startup): in that case the
        rise cannot be measured and the onset is "undetermined"."""
        past = [float(r[ci]) for r in self._ratioR if np.isfinite(r[ci])]
        return past[-1] - past[0] if len(past) >= self.nTb + 1 else np.nan

    def _classify(self, ev):
        hz = self.cfg.hz
        chans = {}
        for ci, c in enumerate(CHANNELS):
            if ev["late"][ci]:
                cls = "persistent"
            elif ev["ever"][ci]:
                cls = "transient"
            else:
                cls = "none"
            r = float(ev["rise"][ci])
            if cls == "none":
                onset, rise = None, None
            elif not np.isfinite(r):
                onset, rise = "undetermined", None      # alarm right after startup
            else:
                onset = "abrupt" if ev["abrupt"][ci] else "gradual"
                rise = round(r, 3)
            chans[c] = {"class": cls, "onset": onset, "SR_rise": rise,
                        "persistent_by": (None if cls != "persistent" else
                                          "CUSUM" if ev.get("late_cusum", np.zeros(2, bool))[ci]
                                          else "S_R")}
        persistent = bool(ev["late"].any())
        event = {"t_alarm": int(ev["t_alarm"]), "t_alarm_s": ev["t_alarm"] / hz,
                 "t_decision_s": self.t / hz, "trigger": ev["trigger"],
                 "class": "persistent" if persistent else "transient",
                 "channels": chans}
        self.events.append(event)
        self._persist_ev = event if persistent else None
        self.state = PERSISTENT if persistent else NORMAL
        self._quiet = 0
        return event