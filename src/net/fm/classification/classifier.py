"""
Classification of perturbations by PHYSICAL CAUSE, downstream of the detector.

The detector (detection/detector.py) is NOT modified: it is the trigger. This
module reads what it already produces (fast events, slow events and the
sample-by-sample series returned by det_common.replay) and, for every event:

  [1] episodes_from_replay   builds the analysis window from the onset
  [2] compute_signature      computes the signature (feature dict), normalised
                             with the nominal residual statistics (calibration)
  [3] apply_rules            physical rules -> category + confidence + reasons
  [4] SignatureLibrary       incremental library: "already seen" / new entry,
                             disagreement with the rules -> doubtful

Classes: contact, weight_obstacle, body_change, sensor_fault, unknown.
No training data per class and no model per perturbation: only the frozen FM
(through the detector) and the library learn nothing but prototypes.

Units: every z quantity is a residual divided by the nominal sigma of the
calibration (1 = typical model error). The residual used for amplitude and
regression is the ROLLOUT one (it does not see the perturbation in its input);
the residual used for the channel coherence is the ISOLATED one-step
(resid_iso: each channel predicted with the other channel's history from the
rollout, so a fault on one sensor does not leak into the other channel).

All thresholds are in ClassifierConfig: they are first guesses, to be tuned
on the validation recordings (repeated perturbations, injected faults).
"""
import json
import os
from dataclasses import dataclass, asdict, field

import numpy as np

# same conventions as detector.py
CHANNELS = ("sensor_diff", "current")
M1, MR = 0, 1

LABELS = ("contact", "weight_obstacle", "body_change", "sensor_fault", "unknown")


# =============================================================================
# configuration
# =============================================================================
@dataclass
class ClassifierConfig:
    # ---- [1] analysis window ----
    pre_s: float = 2.0             # max look-back before the alarm to find the onset
    onset_frac: float = 0.5        # onset = first sample with S_R/threshold above this
    persist_window_s: float = 6.0  # window after the onset for persistent / slow events
    max_window_s: float = 15.0     # the window can grow up to this to reach enough excitation
    attach_slow_s: float = 5.0     # a slow alarm within this time of a fast event belongs to it
    reclass_transient_s: float = 1.0  # "persistent" event that ends within this time of the
                                      # decision -> treated as transient (S_R was already back
                                      # to normal: persistent only because of the CUSUM rule)
    # ---- excitation ----
    min_cycles: float = 2.0        # command cycles needed in the window
    min_cmd_std: float = 0.3       # minimum std of the command in the window (normalised units)
    # ---- start-of-session check ----
    startup_s: float = 5.0         # seconds after STARTUP used for the check
    startup_z2: float = 3.0        # mean z2 (one-step) above this -> event with onset t=0
    # ---- [2]/[3] channel coherence (isolated residual, RMS of z) ----
    dev_rms: float = 2.0           # channel deviates if rms_iso >= this (fast events)
    nominal_rms: float = 1.3       # channel is nominal if rms_iso <= this
    slow_dev_rms: float = 1.4      # deviation threshold for slow / startup episodes (small changes)
    # ---- [3] rules ----
    mult_dominant: float = 0.6     # mult_ratio above -> multiplicative (scales with the command)
    add_dominant: float = 0.4      # mult_ratio below -> additive
    asym_thr: float = 0.3          # |asymmetry| above -> depends on the command direction
    flat_thr: float = 0.2          # median step measured / nominal below -> stuck sensor
    zero_tol: dict = field(default_factory=lambda: {"sensor_diff": 5.0, "current": 3.0})  # raw units
    noise_hf: float = 0.8          # hf ratio above (and large rms) -> noise
    drift_z: float = 2.0           # |trend| (last third - first third, z) above -> drift ...
    drift_frac: float = 0.5        # ... and |trend| > drift_frac * |mean z| (it grows, not a step)
    # ---- [4] library ----
    tau_known: float = 2.5         # distance below -> "already seen"
    min_n_var: int = 3             # entries with at least n examples use their own variance
    var_floor: float = 0.25        # minimum variance per feature (scaled units)


# =============================================================================
# [1] episodes: analysis windows from the detector output
# =============================================================================
def _n(cfg_hz, seconds):
    return max(1, int(round(seconds * cfg_hz)))


def _excitation(cmd):
    """(std, cycles) of the command in a window. cycles = zero crossings / 2."""
    c = np.asarray(cmd, dtype=np.float64)
    if len(c) < 3:
        return 0.0, 0.0
    d = c - c.mean()
    crossings = np.count_nonzero(np.signbit(d[1:]) != np.signbit(d[:-1]))
    return float(c.std()), crossings / 2.0


def _grow_for_excitation(a, b, cmd, n_max, hz, cfg):
    """Extends the end b of the window [a, b) until there are enough command
    cycles, without exceeding max_window_s or the end of the trial."""
    b_lim = min(n_max, a + _n(hz, cfg.max_window_s))
    while b < b_lim:
        std, cyc = _excitation(cmd[a:b])
        if cyc >= cfg.min_cycles and std >= cfg.min_cmd_std:
            break
        b = min(b_lim, b + _n(hz, 1.0))
    return b


def episodes_from_replay(r, det, cmd, cfg=None):
    """Builds the episodes to classify from the detector output.

    r    dict returned by det_common.replay(det, cmd, sens)
    det  the PerturbationDetector after the replay (det.events, det.slow_events)
    cmd  normalised command (n,)
    Returns a list of dicts:
      kind        "transient" | "persistent" | "slow" | "startup"
      a, b        window [a, b) in samples
      onset, t_onset_s, t_end_s (None if still active at the end of the trial)
      fast        the fast event dict (or None)
      slow        list of slow event dicts attached to the episode
    """
    cfg = cfg or ClassifierConfig()
    hz = det.cfg.hz
    n = len(cmd)
    thr = det.calib.threshold[MR]
    ratio = r["score"][:, MR, :] / thr                       # (n,2) S_R/threshold
    rmax = np.nan_to_num(np.fmax(ratio[:, 0], ratio[:, 1]), nan=0.0)   # NaN during startup
    episodes = []
    used_slow = set()

    # ---- fast events ----
    for ev in det.events:
        t0 = int(ev["t_alarm"])
        a = t0
        lim = max(0, t0 - _n(hz, cfg.pre_s))
        while a > lim and rmax[a - 1] > cfg.onset_frac:      # walk back to the onset
            a -= 1
        a = max(0, a - det.nW)                               # the score lags by the window W
        t_dec = int(round(ev["t_decision_s"] * hz))
        kind = ev["class"]
        reclassified = False
        if (kind == "persistent" and ev.get("t_end_s") is not None
                and ev["t_end_s"] - ev["t_decision_s"] <= cfg.reclass_transient_s):
            kind, reclassified = "transient", True
        if kind == "transient":
            above = np.flatnonzero(rmax[t0:t_dec + 1] > 1.0)
            b = t0 + (int(above[-1]) if len(above) else 0) + det.nW + 1
            t_end = b / hz
        else:
            b = _grow_for_excitation(a, a + _n(hz, cfg.persist_window_s), cmd, n, hz, cfg)
            t_end = ev.get("t_end_s")
        b = min(n, max(b, a + 2))
        end_for_slow = (t_end if t_end is not None else n / hz)
        slow = []
        for k, sev in enumerate(det.slow_events):
            ts = sev["t_alarm_s"]
            if a / hz - cfg.attach_slow_s <= ts <= end_for_slow + cfg.attach_slow_s:
                slow.append(sev)
                used_slow.add(k)
        episodes.append({"kind": kind, "a": a, "b": b, "onset": a,
                         "t_onset_s": a / hz, "t_end_s": t_end, "fast": ev, "slow": slow,
                         "detector_class": ev["class"], "reclassified": reclassified})

    # ---- slow events not attached to a fast event ----
    g = r["g"]
    for k, sev in enumerate(det.slow_events):
        if k in used_slow:
            continue
        ci = CHANNELS.index(sev["channel"])
        t0 = int(sev["t_alarm"])
        # CUSUM change point: last instant before the alarm where g was 0
        zeros = np.flatnonzero(np.nan_to_num(g[:t0 + 1, ci], nan=0.0) <= 0.0)
        onset = int(zeros[-1]) if len(zeros) else 0
        # secondary alarm of an episode already open -> attach to it
        host = None
        for ep in episodes:
            if ep["kind"] == "slow" and ep["a"] <= t0 <= (ep["b"] + _n(hz, cfg.attach_slow_s)):
                host = ep
                break
        if host is not None:
            host["slow"].append(sev)
            continue
        # window: from the change point, but at most the last max_window_s
        # before the alarm (with a high h the CUSUM can take a long time and
        # the stretch from the change point would contain other events)
        a = max(onset, t0 + 1 - _n(hz, cfg.max_window_s))
        b = max(t0 + 1, a + _n(hz, cfg.persist_window_s))
        b = _grow_for_excitation(a, b, cmd, n, hz, cfg)
        episodes.append({"kind": "slow", "a": a, "b": min(n, b), "onset": onset,
                         "t_onset_s": onset / hz, "t_end_s": sev.get("t_end_s"),
                         "fast": None, "slow": [sev]})

    episodes.sort(key=lambda e: e["a"])
    return episodes


def startup_episode(r, det, cmd, cfg=None):
    """Start-of-session check: if the one-step residual is already out of
    distribution right after STARTUP (e.g. fin missing from power-on), returns
    an episode with onset at t=0, otherwise None."""
    cfg = cfg or ClassifierConfig()
    hz = det.cfg.hz
    valid = np.flatnonzero(np.isfinite(r["resid"][:, M1, 0]))
    if len(valid) == 0:
        return None
    a = int(valid[0])
    b = min(len(cmd), a + _n(hz, cfg.startup_s))
    z2 = (r["resid"][a:b, M1, :] / det.calib.sigma[M1]) ** 2
    mz2 = np.nanmean(z2, axis=0)
    if not (mz2 > cfg.startup_z2).any():
        return None
    b = _grow_for_excitation(a, b, cmd, len(cmd), hz, cfg)
    return {"kind": "startup", "a": a, "b": b, "onset": 0, "t_onset_s": 0.0,
            "t_end_s": None, "fast": None, "slow": [],
            "startup_mean_z2": {c: float(mz2[i]) for i, c in enumerate(CHANNELS)}}


# =============================================================================
# [2] signature
# =============================================================================
def _linfit(x, y):
    """y ~ a*x + b by least squares. Returns (a, b, R2)."""
    X = np.column_stack([x, np.ones_like(x)])
    coef, *_ = np.linalg.lstsq(X, y, rcond=None)
    pred = X @ coef
    ss_tot = float(np.sum((y - y.mean()) ** 2))
    r2 = 1.0 - float(np.sum((y - pred) ** 2)) / ss_tot if ss_tot > 0 else 0.0
    return float(coef[0]), float(coef[1]), r2


def _asym(z, mask):
    """(rms on the True side - rms on the False side) / sum: 0 = symmetric,
    +-1 = residual on one side only."""
    if mask.sum() < 3 or (~mask).sum() < 3:
        return 0.0
    p = float(np.sqrt(np.mean(z[mask] ** 2)))
    q = float(np.sqrt(np.mean(z[~mask] ** 2)))
    return (p - q) / (p + q) if (p + q) > 0 else 0.0


def _flat(meas, nominal):
    dm = np.median(np.abs(np.diff(meas)))
    dn = np.median(np.abs(np.diff(nominal)))
    return float(dm / dn) if dn > 1e-9 else 1.0


def compute_signature(ep, r, cmd, sens, det, norm_stats=None, cfg=None):
    """Signature of one episode (plain dict, JSON serialisable).

    Detector part: kind, trigger, per-channel class/onset, slow channels.
    Per channel c (z = residual / nominal sigma):
      mean_z, rms_z, peak_z      rollout residual: sign and amplitude
      rms_iso                    isolated one-step residual: channel coherence
      gain_a, offset_bz, fit_r2  regression r = a*u + b, with u = nominal
                                 response (FM rollout prediction, de-meaned):
                                 a = relative gain change (multiplicative),
                                 b/sigma = additive part
      mult_ratio                 |a|*std(u) / (|a|*std(u) + |b|): 1 = purely
                                 multiplicative, 0 = purely additive
      asym_side, asym_vel        dependence on the command direction (tail side
                                 w.r.t. the window centre, sign of the command
                                 velocity)
      flat                       median |step| of the measured signal / median |step|
                                 of the nominal one: ~0 = stuck sensor (robust to
                                 the few samples before the onset)
      hf                         rms(diff z) / (sqrt2 * rms z): ~1 white noise,
                                 ~0 slow offset/drift
      slope_z                    linear trend of z over the window (z per second)
      trend_z                    mean z of the last third minus the first third
      raw_mean                   median measured value in physical units (zero
                                 check; None without norm_stats)
    Window: excitation (command std and cycles), duration, excitation_ok.
    """
    cfg = cfg or ClassifierConfig()
    hz = det.cfg.hz
    a, b = int(ep["a"]), int(ep["b"])
    sigR = det.calib.sigma[MR]
    zR = r["resid"][a:b, MR, :] / sigR
    predR = r["pred"][a:b, MR, :]
    meas = np.asarray(sens[a:b], dtype=np.float64)
    c_win = np.asarray(cmd[a:b], dtype=np.float64)

    # isolated residual (fallback: one-step) normalised with the slow sigma
    iso = r["resid_iso"][a:b]
    if not np.isfinite(iso).all():
        iso = r["resid"][a:b, M1, :]
    iso_sigma = np.asarray(det.cusum_sigma, dtype=np.float64)
    ziso = iso / iso_sigma

    valid = np.isfinite(zR).all(axis=1) & np.isfinite(predR).all(axis=1)
    zR, predR, meas, ziso_v, c_v = zR[valid], predR[valid], meas[valid], ziso[valid], c_win[valid]
    t = np.arange(len(zR)) / hz

    std_c, cycles = _excitation(c_win)
    sig = {
        "kind": ep["kind"],
        "t_onset_s": float(ep["t_onset_s"]),
        "t_end_s": ep.get("t_end_s"),
        "window_s": float((b - a) / hz),
        "n_samples": int(valid.sum()),
        "cmd_std": std_c,
        "cmd_cycles": cycles,
        "excitation_ok": bool(cycles >= cfg.min_cycles and std_c >= cfg.min_cmd_std),
        "fast": ep["fast"] is not None,
        "detector_class": ep.get("detector_class"),
        "reclassified_transient": bool(ep.get("reclassified", False)),
        "slow_channels": sorted({s["channel"] for s in ep["slow"]}),
        "slow_origin": next((s["origin"] for s in ep["slow"] if not s["secondary"]), None),
        "trigger": list(ep["fast"]["trigger"]) if ep["fast"] else [],
        "fast_channels": ({c: ep["fast"]["channels"][c] for c in CHANNELS} if ep["fast"] else {}),
        "channels": {},
    }
    if "startup_mean_z2" in ep:
        sig["startup_mean_z2"] = ep["startup_mean_z2"]
    if len(zR) < 5:
        sig["too_short"] = True
        return sig

    side = (c_v - c_v.mean()) > 0
    vel = np.gradient(c_v) > 0
    for ci, ch in enumerate(CHANNELS):
        z = zR[:, ci]
        u = predR[:, ci] - predR[:, ci].mean()
        res = z * sigR[ci]                                   # normalised units
        ga, gb, r2 = _linfit(u, res)
        mult = abs(ga) * float(u.std())
        add = abs(gb)
        rms = float(np.sqrt(np.mean(z ** 2)))
        dz = np.diff(z)
        slope, _, _ = _linfit(t, z) if len(t) > 2 else (0.0, 0.0, 0.0)
        raw_mean = None                      # physical median: needs norm_stats
        if norm_stats is not None:
            key = "sd" if ch == "sensor_diff" else "vf"
            raw_mean = float(np.median(meas[:, ci])) * norm_stats[f"{key}_std"] + norm_stats[f"{key}_mean"]
        k3 = max(1, len(z) // 3)
        trend = float(z[-k3:].mean() - z[:k3].mean())
        sig["channels"][ch] = {
            "mean_z": float(z.mean()),
            "rms_z": rms,
            "peak_z": float(np.max(np.abs(z))),
            "rms_iso": float(np.sqrt(np.mean(ziso_v[:, ci] ** 2))),
            "gain_a": ga,
            "offset_bz": gb / sigR[ci],
            "fit_r2": r2,
            "mult_ratio": mult / (mult + add) if (mult + add) > 0 else 0.0,
            "asym_side": _asym(z, side),
            "asym_vel": _asym(z, vel),
            "flat": _flat(meas[:, ci], predR[:, ci]),
            "hf": (float(np.sqrt(np.mean(dz ** 2))) / (np.sqrt(2) * rms)) if rms > 0 else 0.0,
            "slope_z": slope,
            "trend_z": trend,
            "raw_mean": raw_mean,
        }
    zc = zR - zR.mean(axis=0)
    den = np.sqrt((zc ** 2).sum(axis=0).prod())
    sig["corr_channels"] = float((zc[:, 0] * zc[:, 1]).sum() / den) if den > 0 else 0.0
    return sig


# =============================================================================
# [3] physical rules
# =============================================================================
def _clip01(x):
    return float(min(1.0, max(0.0, x)))


def _sensor_fault_subtype(ch, c, sig, cfg):
    """stuck / stuck_at_zero / noise / drift / offset / gain / other."""
    if c["flat"] < cfg.flat_thr:
        if c["raw_mean"] is not None and abs(c["raw_mean"]) < cfg.zero_tol[ch]:
            return "stuck_at_zero"
        return "stuck"
    if c["hf"] > cfg.noise_hf and abs(c["mean_z"]) < 0.5 * c["rms_z"]:
        return "noise"
    if abs(c["trend_z"]) > cfg.drift_z and abs(c["trend_z"]) > cfg.drift_frac * abs(c["mean_z"]):
        return "drift"
    if abs(c["mean_z"]) >= 0.7 * c["rms_z"]:
        return "offset"
    if c["mult_ratio"] >= cfg.mult_dominant:
        return "gain"
    return "other"


def apply_rules(sig, cfg=None):
    """Physical rules -> {"label", "subtype", "confidence", "status", "reasons"}.
    status: "classified" | "postponed" (not enough command excitation for a
    persistent/slow/startup episode) | "too_short".
    The confidence is a heuristic in [0, 1] (margins of the deciding features),
    not a probability."""
    cfg = cfg or ClassifierConfig()
    out = {"label": "unknown", "subtype": None, "confidence": 0.0,
           "status": "classified", "reasons": []}
    if sig.get("too_short"):
        out.update(status="too_short", reasons=["window too short"])
        return out
    kind = sig["kind"]
    if kind != "transient" and not sig["excitation_ok"]:
        out.update(status="postponed",
                   reasons=[f"command excitation too low ({sig['cmd_cycles']:.1f} cycles, "
                            f"std {sig['cmd_std']:.2f})"])
        return out

    ch = sig["channels"]
    dev_thr = cfg.dev_rms if kind in ("transient", "persistent") else cfg.slow_dev_rms
    rms = {c: ch[c]["rms_iso"] for c in CHANNELS}
    dev = [c for c in CHANNELS if rms[c] >= dev_thr]
    nominal = [c for c in CHANNELS if rms[c] <= cfg.nominal_rms]
    R = out["reasons"]
    R.append("isolated rms: " + ", ".join(f"{c} {rms[c]:.2f}" for c in CHANNELS)
             + f" (deviates >= {dev_thr}, nominal <= {cfg.nominal_rms})")

    if not dev:
        R.append("no channel clearly deviates")
        return out

    # ---- only one channel deviates, the other is nominal -> sensor fault ----
    if len(dev) == 1 and len(nominal) == 1 and nominal[0] != dev[0]:
        c, o = dev[0], nominal[0]
        sub = _sensor_fault_subtype(c, ch[c], sig, cfg)
        conf = 0.5 * _clip01((rms[c] - dev_thr) / dev_thr + 0.5) \
             + 0.5 * _clip01((cfg.nominal_rms - rms[o]) / cfg.nominal_rms + 0.5)
        R.append(f"only {c} deviates, {o} consistent with the nominal model")
        out.update(label="sensor_fault", subtype=f"{c}:{sub}", confidence=conf)
        return out

    # ---- both channels involved ----
    main = max(CHANNELS, key=lambda c: rms[c])
    m = ch[main]
    R.append(f"both channels involved (main: {main})")
    if kind == "transient":
        conf = _clip01(0.5 + 0.1 * (m["peak_z"] - dev_thr))
        R.append(f"transient, peak {m['peak_z']:.1f} z"
                 + (" (detector said persistent, but it ended right after the decision)"
                    if sig.get("reclassified_transient") else ""))
        out.update(label="contact", confidence=conf)
        return out

    asym = max(abs(ch[c]["asym_side"]) for c in CHANNELS)
    mr = m["mult_ratio"]
    R.append(f"persistent: mult_ratio {mr:.2f}, max |asymmetry| {asym:.2f}")
    if mr >= cfg.mult_dominant and asym < cfg.asym_thr:
        conf = _clip01(0.5 + (mr - cfg.mult_dominant) + (0.2 if kind == "slow" else 0.0))
        R.append("multiplicative (scales with the command)"
                 + (", from the slow detector" if kind in ("slow", "startup") else ""))
        out.update(label="body_change", confidence=conf)
    elif mr <= cfg.add_dominant or asym >= cfg.asym_thr:
        conf = _clip01(0.5 + (cfg.add_dominant - mr) + (asym - cfg.asym_thr))
        R.append("additive" + (" and direction-dependent" if asym >= cfg.asym_thr else ""))
        out.update(label="weight_obstacle", confidence=conf)
    else:
        R.append("neither clearly additive nor multiplicative")
    return out


# =============================================================================
# [4] incremental library of signatures
# =============================================================================
# features used for the distance, with their scale (value / scale -> ~unit spread)
VECTOR_SPEC = (
    ("persistent", None, 1.0),
    ("slow_only", None, 1.0),
) + tuple(
    (f"{c}:{k}", c, s) for c in CHANNELS for k, s in (
        ("mean_z", 3.0), ("log_rms", 0.5), ("gain_a", 0.3), ("offset_bz", 3.0),
        ("asym_side", 0.5), ("flat", 0.5), ("hf", 0.5), ("iso_ratio", 0.5)))


def signature_vector(sig):
    """Numeric vector (already scaled) used by the library."""
    v = []
    for name, ch, scale in VECTOR_SPEC:
        if name == "persistent":
            x = 0.0 if sig["kind"] == "transient" else 1.0
        elif name == "slow_only":
            x = 1.0 if (sig["kind"] in ("slow", "startup")) else 0.0
        else:
            c = sig["channels"][ch]
            key = name.split(":")[1]
            if key == "log_rms":
                x = np.log1p(c["rms_z"])
            elif key == "flat":
                x = min(c["flat"], 2.0)
            elif key == "iso_ratio":       # share of the channel in the isolated deviation
                tot = sum(sig["channels"][k]["rms_iso"] for k in CHANNELS)
                x = c["rms_iso"] / tot if tot > 0 else 0.5
            else:
                x = c[key]
        v.append(float(x) / scale)
    return np.array(v)


@dataclass
class LibraryEntry:
    id: int
    label: str                    # label at creation (rules, or "unknown #k")
    user_label: str = None        # manual rename (overrides label in the output)
    n: int = 0
    mean: list = None             # prototype (scaled vector)
    m2: list = None               # Welford sum of squares, for the variance
    rule_labels: dict = field(default_factory=dict)   # label -> count, from the rules
    history: list = field(default_factory=list)       # [{source, t_onset_s, t_end_s, distance}]
    active: bool = False          # a persistent episode of this entry is in progress

    @property
    def name(self):
        return self.user_label or self.label

    def distance(self, v, cfg):
        mu = np.asarray(self.mean)
        if self.n >= cfg.min_n_var:
            var = np.maximum(np.asarray(self.m2) / (self.n - 1), cfg.var_floor)
        else:
            var = np.ones_like(mu)
        return float(np.sqrt(np.mean((v - mu) ** 2 / var)))

    def update(self, v):
        if self.mean is None:
            self.n, self.mean, self.m2 = 1, v.tolist(), np.zeros_like(v).tolist()
            return
        self.n += 1
        mu, m2 = np.asarray(self.mean), np.asarray(self.m2)
        d = v - mu
        mu = mu + d / self.n
        m2 = m2 + d * (v - mu)
        self.mean, self.m2 = mu.tolist(), m2.tolist()


class SignatureLibrary:
    """Incremental library: every new signature is compared with the
    prototypes. Close -> "known" (prototype updated if update=True); far from
    all of them -> new entry, labelled by the rules or "unknown #k".
    Rules and library disagree -> doubtful.

        lib = SignatureLibrary.load("library.json", cfg)   # or SignatureLibrary(cfg)
        res = lib.process(sig, rule, source="trial_3.csv")
        lib.save("library.json")
    """

    def __init__(self, cfg=None):
        self.cfg = cfg or ClassifierConfig()
        self.entries = []
        self._unknown_k = 0

    # ---------------------------------------------------------------- lookup
    def nearest(self, v):
        if not self.entries:
            return None, np.inf
        d = [e.distance(v, self.cfg) for e in self.entries]
        i = int(np.argmin(d))
        return self.entries[i], d[i]

    def process(self, sig, rule, source="", update=True):
        """Returns {"entry_id", "entry_name", "status": "known"|"new"|"skipped",
        "distance", "doubtful", "doubt_reason"}."""
        res = {"entry_id": None, "entry_name": None, "status": "skipped",
               "distance": None, "doubtful": False, "doubt_reason": None}
        if rule["status"] != "classified":
            return res
        v = signature_vector(sig)
        entry, d = self.nearest(v)
        rl = rule["label"]
        if entry is not None and d < self.cfg.tau_known:
            res.update(status="known", distance=d)
            base = entry.label.split(" #")[0]
            if rl != "unknown" and base != "unknown" and rl != base:
                res.update(doubtful=True,
                           doubt_reason=f"rules say '{rl}', library entry is '{entry.name}'")
            if update:
                entry.update(v)
        else:
            if entry is not None:
                res["distance"] = d
            if rl == "unknown":
                self._unknown_k += 1
                label = f"unknown #{self._unknown_k}"
            else:
                label = rl
            entry = LibraryEntry(id=len(self.entries) + 1, label=label)
            entry.update(v)
            res["status"] = "new"
            if update:
                self.entries.append(entry)
        entry.rule_labels[rl] = entry.rule_labels.get(rl, 0) + 1
        entry.history.append({"source": source, "t_onset_s": sig["t_onset_s"],
                              "t_end_s": sig.get("t_end_s"), "distance": res["distance"]})
        # persistent episode still open at the end of the data -> entry active
        entry.active = sig["kind"] != "transient" and sig.get("t_end_s") is None
        res.update(entry_id=entry.id, entry_name=entry.name)
        return res

    def close_active(self, entry_id, t_end_s):
        """Return to nominal of a persistent episode (perturbation removed)."""
        for e in self.entries:
            if e.id == entry_id and e.active:
                e.active = False
                if e.history:
                    e.history[-1]["t_end_s"] = t_end_s

    def rename(self, entry_id, user_label):
        for e in self.entries:
            if e.id == entry_id:
                e.user_label = user_label

    # ---------------------------------------------------------------- tau
    @staticmethod
    def suggest_tau(groups, cfg=None):
        """Threshold "new vs already seen" from repeated perturbations.
        groups: {name: [signature, ...]} with the same perturbation repeated.
        Returns (suggested tau, table) with, per group, the largest distance
        of a repetition from its own prototype and the smallest distance from
        the prototypes of the other groups. tau between the two = separable."""
        cfg = cfg or ClassifierConfig()
        protos, table = {}, []
        for name, sigs in groups.items():
            e = LibraryEntry(id=0, label=name)
            for s in sigs:
                e.update(signature_vector(s))
            protos[name] = e
        within_max = 0.0
        between_min = np.inf
        for name, sigs in groups.items():
            vs = [signature_vector(s) for s in sigs]
            w = max(protos[name].distance(v, cfg) for v in vs)
            others = [protos[o].distance(v, cfg) for o in protos if o != name for v in vs]
            bmin = min(others) if others else np.inf
            within_max, between_min = max(within_max, w), min(between_min, bmin)
            table.append({"group": name, "n": len(vs), "within_max": w, "between_min": bmin})
        tau = (within_max + between_min) / 2 if np.isfinite(between_min) else 1.2 * within_max
        return tau, table

    # ---------------------------------------------------------------- I/O
    def save(self, path):
        with open(path, "w") as f:
            json.dump({"config": asdict(self.cfg), "unknown_k": self._unknown_k,
                       "vector_spec": [s[0] for s in VECTOR_SPEC],
                       "entries": [asdict(e) for e in self.entries]}, f, indent=2)

    @staticmethod
    def load(path, cfg=None):
        lib = SignatureLibrary(cfg)
        if not os.path.exists(path):
            return lib
        with open(path) as f:
            d = json.load(f)
        if d.get("vector_spec") != [s[0] for s in VECTOR_SPEC]:
            raise ValueError(f"{path}: library built with a different feature vector")
        lib._unknown_k = d.get("unknown_k", 0)
        lib.entries = [LibraryEntry(**e) for e in d["entries"]]
        return lib

    def summary(self):
        lines = []
        for e in self.entries:
            rl = ", ".join(f"{k} {v}" for k, v in sorted(e.rule_labels.items()))
            lines.append(f"  #{e.id:<3d} {e.name:22s} n={e.n:<3d} rules: {rl}"
                         + ("  [ACTIVE]" if e.active else ""))
        return "\n".join(lines) if lines else "  (empty)"


# =============================================================================
# software sensor faults (validation on nominal logs)
# =============================================================================
FAULT_TYPES = ("stuck", "offset", "drift", "noise", "zero")


def inject_sensor_fault(sens, ch, kind, t0_s, value, hz=10.0, norm_stats=None, rng=None):
    """Returns a copy of sens (n,2), NORMALISED, with a sensor fault on channel
    ch from t0_s to the end. value is in PHYSICAL units if norm_stats is given
    (sensor units / mA), otherwise in normalised units:
      stuck   the sensor keeps the value it had at t0 (value ignored)
      offset  + value
      drift   + value per second, from t0
      noise   + white noise with std = value
      zero    the sensor reads 0 (physical zero; value ignored)
    """
    s = np.array(sens, dtype=np.float64, copy=True)
    ci = CHANNELS.index(ch)
    i0 = int(round(t0_s * hz))
    if i0 >= len(s):
        return s
    std = 1.0
    if norm_stats is not None:
        key = "sd" if ch == "sensor_diff" else "vf"
        std = float(norm_stats[f"{key}_std"])
        zero_norm = -float(norm_stats[f"{key}_mean"]) / std
    else:
        zero_norm = 0.0
    v = value / std
    rng = rng or np.random.default_rng(0)
    if kind == "stuck":
        s[i0:, ci] = s[i0, ci]
    elif kind == "offset":
        s[i0:, ci] += v
    elif kind == "drift":
        s[i0:, ci] += v * np.arange(len(s) - i0) / hz
    elif kind == "noise":
        s[i0:, ci] += rng.normal(0.0, v, len(s) - i0)
    elif kind == "zero":
        s[i0:, ci] = zero_norm
    else:
        raise ValueError(f"fault type must be one of {FAULT_TYPES}")
    return s