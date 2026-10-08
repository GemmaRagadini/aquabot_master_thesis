"""
Detector ONLINE di perturbazioni basato sul forward model (FM) congelato.

A ogni campione (10 Hz) il FM predice sens(t) in due modi:
  modo "1" (un passo) : storia sensori VERA      t-H .. t-1
  modo "R" (rollout)  : storia sensori PREDETTA  (il FM non rivede i sensori
                        reali finche' il rollout non viene riagganciato)
Per ogni modo m e canale c:
  residuo   e = y - y_pred
  score     S(t) = sqrt( 1/W * sum_{k<W} (e(t-k) / sigma)^2 )
sigma e soglie arrivano dalla calibrazione (calibrate.py) e stanno in un JSON.

Macchina a stati:
  AVVIO       riempie la storia con H campioni reali
  NORMALE     calcola gli score; riaggancia il rollout ai dati reali se tutto
              e' quieto da quiet_s e l'ultimo riaggancio e' piu' vecchio di
              min_reanchor_s
  ALLARME     S_R di un canale ha superato la soglia a t_a; dopo T secondi
              classifica canale per canale:
                persistente  S_R sopra soglia negli ultimi Tc secondi
                transitoria  S_R sopra soglia solo prima
                nessuna      canale mai sopra soglia
              ed etichetta l'inizio: "brusco" se S_1 supera la sua soglia
              entro Tb da t_a, altrimenti "graduale"
  PERSISTENTE continua a calcolare gli score; quando S_R di tutti i canali
              resta sotto soglia per quiet_s torna da solo a NORMALE e
              registra fine e durata nell'evento. reset() resta disponibile
              per forzare l'uscita a mano.

Tutti gli array 2x2 sono indicizzati [modo, canale] con MODES e CHANNELS.
Questo file dipende solo da numpy e torch: si puo' portare sul robot.
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

AVVIO, NORMALE, ALLARME, PERSISTENTE = "avvio", "normale", "allarme", "persistente"


@dataclass
class DetectorConfig:
    hz:             float = 10.0   # frequenza dei campioni
    W_s:            float = 0.5    # finestra dello score
    T_s:            float = 2.5    # attesa tra allarme e classificazione
    Tc_s:           float = 1.0    # ultimo tratto di T: S_R ancora sopra soglia?
    Tb_s:           float = 0.7    # finestra per l'etichetta "inizio brusco"
    quiet_s:        float = 5.0    # quiete richiesta prima del riaggancio
    min_reanchor_s: float = 20.0   # intervallo minimo tra i riagganci

    def n(self, seconds):
        """Secondi -> numero di campioni (almeno 1)."""
        return max(1, int(round(seconds * self.hz)))


@dataclass
class Calibration:
    sigma:     np.ndarray                      # (2,2) [modo, canale]
    threshold: np.ndarray                      # (2,2) [modo, canale]
    config:    DetectorConfig = field(default_factory=DetectorConfig)
    meta:      dict = field(default_factory=dict)

    @staticmethod
    def neutral(config=None):
        """sigma=1 e soglie infinite: il detector non va mai in allarme e lo
        score e' l'RMS grezzo del residuo. Serve a calibrate.py."""
        return Calibration(np.ones((2, 2)), np.full((2, 2), np.inf),
                           config or DetectorConfig())

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
                       "config": asdict(self.config),
                       "meta": self.meta}, f, indent=2)

    @staticmethod
    def load(path):
        with open(path) as f:
            d = json.load(f)
        return Calibration(Calibration._from_named(d["sigma"]),
                           Calibration._from_named(d["threshold"]),
                           DetectorConfig(**d["config"]), d.get("meta", {}))


class PerturbationDetector:
    """Uso:
        det = PerturbationDetector(FM, Calibration.load("calib.json"), H=20)
        for cmd, sens in stream:            # valori NORMALIZZATI
            out = det.step(cmd, sens)
            if out["alarm"]: ...            # allarme appena scattato
            if out["event"]: ...            # classificazione appena decisa
            if out["cleared"]: ...          # perturbazione persistente finita
    Sul robot, con valori fisici: passare norm_stats (dal checkpoint) e usare
    step_raw(cmd_rad, sensor_diff, current_ma).
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
            raise ValueError("Tc_s deve essere <= T_s")
        if self.nQuiet < self.H + self.nW:
            warnings.warn(
                f"quiet_s ({self.nQuiet} campioni) < H + W ({self.H + self.nW}): "
                f"al riaggancio la storia reale puo' contenere campioni non quieti.")
        self.start()

    # ------------------------------------------------------------------ stato
    def start(self):
        """Azzera tutto: da chiamare all'inizio di ogni trial/sessione."""
        self.t = 0                       # indice del prossimo campione
        self.state = AVVIO
        self.events = []                 # eventi gia' classificati
        self.reanchor_times = []         # indici campione dei riagganci
        self._boot_cmd, self._boot_sens = [], []
        self.hist_cmd = self.hist_real = self.hist_roll = None
        self._z = deque(maxlen=self.nW)          # residui normalizzati (2,2)
        self._over1 = deque(maxlen=self.nW)      # S_1 sopra soglia, recenti (2,)
        self._age = 0                    # campioni dall'ultimo riaggancio
        self._quiet = 0                  # campioni quieti consecutivi
        self._ev = None                  # evento in corso (stato ALLARME)
        self._persist_ev = None          # evento persistente ancora aperto

    def reset(self, reanchor=False):
        """Reset MANUALE (facoltativo): forza il ritorno a NORMALE da
        PERSISTENTE o ALLARME. reanchor=True riaggancia subito il rollout ai
        sensori reali: da usare solo se la perturbazione e' davvero finita."""
        if self.state == AVVIO:
            return
        self.state, self._ev, self._persist_ev, self._quiet = NORMALE, None, None, 0
        if reanchor:
            self._reanchor()

    def _reanchor(self):
        self.hist_roll = self.hist_real.clone()
        self._age = 0
        self.reanchor_times.append(self.t)

    # ------------------------------------------------------------------- step
    def step_raw(self, cmd_rad, sensor_diff, current_ma):
        """Come step(), ma con valori fisici: normalizza con norm_stats."""
        ns = self.norm_stats
        if ns is None:
            raise ValueError("step_raw richiede norm_stats (dal checkpoint).")
        return self.step((cmd_rad - ns["cmd_mean"]) / ns["cmd_std"],
                         [(sensor_diff - ns["sd_mean"]) / ns["sd_std"],
                          (current_ma - ns["vf_mean"]) / ns["vf_std"]])

    @torch.no_grad()
    def step(self, cmd, sens):
        """Un campione: cmd (float) e sens = [sensor_diff, current], NORMALIZZATI.
        Ritorna un dict; pred/resid/score sono (2,2) [modo, canale], NaN in AVVIO
        (score NaN anche finche' la finestra W non e' piena)."""
        cmd_t = torch.tensor([[float(cmd)]], dtype=torch.float32, device=self.device)
        y = torch.as_tensor(np.asarray(sens, dtype=np.float32).reshape(1, 2),
                            device=self.device)
        nan = np.full((2, 2), np.nan)
        out = {"t": self.t, "state": self.state, "pred": nan, "resid": nan,
               "score": nan, "age": 0, "alarm": False, "reanchored": False,
               "event": None, "cleared": None}

        # ---- AVVIO: si riempie la storia con H campioni reali ----
        if self.hist_cmd is None:
            self._boot_cmd.append(cmd_t)
            self._boot_sens.append(y)
            if len(self._boot_cmd) == self.H:
                self.hist_cmd  = torch.cat(self._boot_cmd, dim=0)     # (H,1)
                self.hist_real = torch.cat(self._boot_sens, dim=0)    # (H,2)
                self.hist_roll = self.hist_real.clone()
                self._age = 0
                self.state = NORMALE          # vale dal prossimo campione
            self.t += 1
            return out

        # ---- due predizioni in un solo forward: riga 0 = un passo, 1 = rollout ----
        seq_cmd  = torch.stack([self.hist_cmd, self.hist_cmd])        # (2,H,1)
        seq_sens = torch.stack([self.hist_real, self.hist_roll])      # (2,H,2)
        pred = self.FM(seq_cmd, seq_sens, cmd_t.expand(2, -1))        # (2,2)
        resid = (y - pred).cpu().numpy().astype(np.float64)           # (2,2)

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
        over_R, over_1 = over[MR], over[M1]
        self._over1.append(over_1.copy())

        # ---- macchina a stati ----
        if self.state == PERSISTENTE:
            # uscita automatica: S_R di tutti i canali sotto soglia per quiet_s
            self._quiet = self._quiet + 1 if (valid and not over_R.any()) else 0
            if self._quiet >= self.nQuiet:
                ev = self._persist_ev
                t_end = self.t - self.nQuiet + 1      # inizio del tratto quieto
                ev["t_end_s"] = t_end / self.cfg.hz
                ev["durata_s"] = (t_end - ev["t_alarm"]) / self.cfg.hz
                out["cleared"] = ev
                self.state, self._persist_ev, self._quiet = NORMALE, None, 0
        elif self.state == NORMALE:
            if over_R.any():
                self.state = ALLARME
                out["alarm"] = True
                self._quiet = 0
                self._ev = {
                    "t_alarm": self.t,
                    "trigger": [c for ci, c in enumerate(CHANNELS) if over_R[ci]],
                    "ever": over_R.copy(),
                    "late": np.zeros(2, dtype=bool),
                    # S_1 reagisce prima di S_R: si guardano anche gli ultimi W
                    # campioni PRIMA di t_a, oltre ai Tb successivi.
                    "abrupt": np.any(np.stack(self._over1), axis=0),
                }
            elif valid and not over.any():
                self._quiet += 1
            else:
                self._quiet = 0

        if self.state == ALLARME:
            ev = self._ev
            k = self.t - ev["t_alarm"]
            ev["ever"] |= over_R
            if k <= self.nTb:
                ev["abrupt"] |= over_1
            if k > self.nT - self.nTc:
                ev["late"] |= over_R
            if k >= self.nT:
                out["event"] = self._classify(ev)
                self._ev = None

        # ---- riaggancio del rollout: solo in NORMALE, quieto e non troppo spesso ----
        if (self.state == NORMALE and self._quiet >= self.nQuiet
                and self._age >= self.nMin):
            self._reanchor()
            out["reanchored"] = True

        out.update(state=self.state, pred=pred.cpu().numpy(), resid=resid,
                   score=score, age=self._age)
        self.t += 1
        return out

    def _classify(self, ev):
        hz = self.cfg.hz
        chans = {}
        for ci, c in enumerate(CHANNELS):
            if ev["late"][ci]:
                cls = "persistente"
            elif ev["ever"][ci]:
                cls = "transitoria"
            else:
                cls = "nessuna"
            onset = None if cls == "nessuna" else ("brusco" if ev["abrupt"][ci] else "graduale")
            chans[c] = {"classe": cls, "inizio": onset}
        persistent = bool(ev["late"].any())
        event = {"t_alarm": int(ev["t_alarm"]), "t_alarm_s": ev["t_alarm"] / hz,
                 "t_decision_s": self.t / hz, "trigger": ev["trigger"],
                 "classe": "persistente" if persistent else "transitoria",
                 "channels": chans}
        self.events.append(event)
        self._persist_ev = event if persistent else None
        self.state = PERSISTENTE if persistent else NORMALE
        self._quiet = 0
        return event