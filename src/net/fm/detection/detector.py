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
              ed etichetta l'inizio di ogni canale dalla velocita' di
              salita di S_R: "brusco" se nei Tb prima del superamento S_R
              e' cresciuto di piu' di rise_frac * soglia, altrimenti
              "graduale". S_1 viene calcolato e registrato, ma non decide
              l'etichetta: entra solo nella condizione di quiete del
              riaggancio.
  PERSISTENTE continua a calcolare gli score; quando S_R di tutti i canali
              resta sotto soglia per quiet_s torna da solo a NORMALE e
              registra fine e durata nell'evento. reset() resta disponibile
              per forzare l'uscita a mano.

Detector LENTO (CUSUM), in parallelo alla macchina a stati, per perturbazioni
piccole ma persistenti (es. pinna mancante) che non superano mai le soglie.
Default (scelto dai dati): residuo A UN PASSO, riferimento FISSO, kappa = 1.
Il riferimento di sessione su 10 s si e' rivelato inaffidabile: l'errore del
modello varia di 10-18 volte tra blocchi di 3 s anche in condizioni normali.
  z2(t)  = min( (e(t)/sigma_C)^2 , cusum_clip )    residuo cusum_mode, per canale
  cusum_mode "1i" (default): un passo, ISOLATO sui canali in cusum_iso. Per
           un canale isolato la sua storia e' reale e quella dell'altro viene
           dal rollout (che non vede perturbazioni): una perturbazione
           sull'altro canale non lo sporca. Default: isolato solo
           sensor_diff (la corrente perturbata sporcava sensor_diff; isolare
           anche la corrente la rende sensibile agli errori del rollout di
           sensor_diff e ne alza molto la soglia). sigma_C = cusum_sigma.
  ref    = media di z2 nei primi cusum_ref_s secondi in stato NORMALE
           (riferimento del robot in QUESTA sessione; congelato dopo)
           cusum_ref_s = 0 -> ref = 1 (riferimento fisso dei trial normali)
  g(t)   = max(0, g(t-1) + z2/ref - 1 - cusum_kappa)
  allarme lento sul canale se g > h (h = cusum_h, da calibrate.py);
  finisce quando g scende sotto cusum_off * h e ci resta per cusum_merge_s
  (se nel frattempo g risupera h e' lo stesso evento che continua); la fine
  registrata e' l'istante della discesa. g e' limitato a 1.2 h.
  Se un allarme lento parte mentre l'altro canale e' gia' in allarme, viene
  marcato "secondario" con il canale di origine: nei test sintetici il canale
  perturbato va sempre in allarme per primo.
  I due detector restano SEPARATI: il veloce classifica le deviazioni grandi
  (transitoria/persistente), il lento segnala un piccolo cambiamento
  persistente in corso. Nota: il lento usa il residuo a un passo, quindi una
  perturbazione su un canale puo' far scattare anche l'altro (la storia
  perturbata sporca entrambe le predizioni): il canale e' indicativo. Non cambia lo stato della macchina a stati.
  NB: con riferimento di sessione rileva cambiamenti RISPETTO ALL'INIZIO della
  sessione: una perturbazione gia' presente all'accensione entra nel riferimento.
  reset_reference() riparte con un nuovo riferimento.

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
SLOW_MODES = ("1i", "1", "R")    # residui possibili per il detector lento

AVVIO, NORMALE, ALLARME, PERSISTENTE = "avvio", "normale", "allarme", "persistente"


@dataclass
class DetectorConfig:
    hz:             float = 10.0   # frequenza dei campioni
    W_s:            float = 0.5    # finestra dello score
    T_s:            float = 6.5    # attesa tra allarme e classificazione (transitoria se < T - Tc - W = 5 s)
    Tc_s:           float = 1.0    # ultimo tratto di T: S_R ancora sopra soglia?
    Tb_s:           float = 0.7    # finestra su cui si misura la salita di S_R
    rise_frac:      float = 0.5    # salita minima (in frazione di soglia) per "brusco"
    quiet_s:        float = 5.0    # quiete richiesta prima del riaggancio
    min_reanchor_s: float = 20.0   # intervallo minimo tra i riagganci
    # --- detector lento (CUSUM) ---
    cusum_mode:     str   = "1i"   # residuo: "1i" un passo, isolato sui canali di cusum_iso
                                   #          (default), "1" un passo, "R" rollout
    cusum_iso:      str   = "sensor_diff"  # canali isolati in modo "1i" (separati da virgola)
    cusum_ref_s:    float = 0.0    # 0 = riferimento fisso (default); >0 = di sessione
    cusum_kappa:    float = 1.0    # g cresce se z2 (clippato) supera in media 1 + kappa
    cusum_clip:     float = 9.0    # limite su z2 di un singolo campione (3 sigma)
    cusum_off:      float = 0.5    # l'allarme lento finisce quando g < cusum_off * h ...
    cusum_merge_s:  float = 5.0    # ... e ci resta per cusum_merge_s (altrimenti continua)

    def n(self, seconds):
        """Secondi -> numero di campioni (almeno 1)."""
        return max(1, int(round(seconds * self.hz)))


def cusum_run(z2, ref_n, kappa, clip):
    """CUSUM offline su una serie z2 (n, 2), stessa matematica del detector
    online (senza il limite 2h). Riferimento = media dei primi ref_n campioni
    (ref_n = 0 -> riferimento 1). Ritorna (g (n,2) con NaN durante il
    riferimento, ref (2,))."""
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
    sigma:     np.ndarray                      # (2,2) [modo, canale]
    threshold: np.ndarray                      # (2,2) [modo, canale]
    config:    DetectorConfig = field(default_factory=DetectorConfig)
    meta:      dict = field(default_factory=dict)
    cusum_h:   np.ndarray = field(default_factory=lambda: np.full(2, np.inf))  # (2,) canale
    cusum_sigma: np.ndarray = None    # (2,) normalizzatore del residuo lento (None = sigma del modo)

    @staticmethod
    def neutral(config=None):
        """sigma=1 e soglie infinite: il detector non va mai in allarme e lo
        score e' l'RMS grezzo del residuo. Serve a calibrate.py."""
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
        # cusum_h assente (JSON vecchi) -> detector lento disattivato
        ch = d.get("cusum_h", {})
        cusum_h = np.array([float(ch.get(c, np.inf)) for c in CHANNELS])
        cs = d.get("cusum_sigma")
        cusum_sigma = None if cs is None else np.array([float(cs[c]) for c in CHANNELS])
        return Calibration(Calibration._from_named(d["sigma"]),
                           Calibration._from_named(d["threshold"]),
                           DetectorConfig(**d["config"]), d.get("meta", {}), cusum_h,
                           cusum_sigma)


class PerturbationDetector:
    """Uso:
        det = PerturbationDetector(FM, Calibration.load("calib.json"), H=20)
        for cmd, sens in stream:            # valori NORMALIZZATI
            out = det.step(cmd, sens)
            if out["alarm"]: ...            # allarme appena scattato
            if out["event"]: ...            # classificazione appena decisa
            if out["cleared"]: ...          # perturbazione persistente finita
            if out["slow_alarm"]: ...       # allarme del detector lento (canali)
            if out["slow_cleared"]: ...     # fine di un allarme lento
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
        if c.cusum_mode not in SLOW_MODES:
            raise ValueError(f"cusum_mode deve essere uno di {SLOW_MODES}")
        self.iso = c.cusum_mode == "1i"
        iso_ch = [x.strip() for x in c.cusum_iso.split(",") if x.strip()]
        bad = [x for x in iso_ch if x not in CHANNELS]
        if bad:
            raise ValueError(f"cusum_iso: canali sconosciuti {bad}")
        self.iso_mask = np.array([ch in iso_ch for ch in CHANNELS])
        self.mC = M1 if self.iso else MODES.index(c.cusum_mode)
        self.cusum_sigma = (np.asarray(calib.cusum_sigma, dtype=np.float64)
                            if calib.cusum_sigma is not None else calib.sigma[self.mC])
        self.nRef = c.n(c.cusum_ref_s) if c.cusum_ref_s > 0 else 0
        self.h = np.asarray(calib.cusum_h, dtype=np.float64)
        self.nMerge = c.n(c.cusum_merge_s)
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
        self._ratioR = deque(maxlen=self.nTb + 1)  # S_R / soglia, ultimi Tb (2,)
        self._age = 0                    # campioni dall'ultimo riaggancio
        self._quiet = 0                  # campioni quieti consecutivi
        self._ev = None                  # evento in corso (stato ALLARME)
        self._persist_ev = None          # evento persistente ancora aperto
        self.slow_events = []            # eventi del detector lento
        self.reset_reference()

    def reset_reference(self):
        """Riparte con un nuovo riferimento per il detector lento: i prossimi
        cusum_ref_s secondi in stato NORMALE diventano il nuovo 'normale'."""
        self._ref_sum = np.zeros(2)
        self._ref_n = 0
        self.ref = None if self.nRef > 0 else np.ones(2)
        self.g = np.zeros(2)
        self._slow_ev = [None, None]
        self._pending_end = [None, None]     # istante in cui g e' sceso sotto off*h

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
               "event": None, "cleared": None,
               "g": np.full(2, np.nan), "slow": np.zeros(2, dtype=bool),
               "resid_iso": np.full(2, np.nan),
               "slow_alarm": [], "slow_cleared": []}

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

        # ---- predizioni in un solo forward: riga 0 = un passo, 1 = rollout,
        #      (se iso) 2 = sensor_diff isolato, 3 = current isolato ----
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
                    "ever": np.zeros(2, dtype=bool),
                    "late": np.zeros(2, dtype=bool),
                    "abrupt": np.zeros(2, dtype=bool),
                    "rise": np.full(2, np.nan),
                }
            elif valid and not over.any():
                self._quiet += 1
            else:
                self._quiet = 0

        if self.state == ALLARME:
            ev = self._ev
            k = self.t - ev["t_alarm"]
            # etichetta dell'inizio: fissata per ogni canale nel momento in cui
            # il SUO S_R supera la soglia per la prima volta
            for ci in np.flatnonzero(over_R & ~ev["ever"]):
                ev["rise"][ci] = self._rise(ci)
                ev["abrupt"][ci] = ev["rise"][ci] > self.cfg.rise_frac
            ev["ever"] |= over_R
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

        e_slow = (np.where(self.iso_mask, out["resid_iso"], resid[M1]) if self.iso
                  else resid[self.mC])
        self._slow_step(e_slow, out)

        out.update(state=self.state, pred=pred.cpu().numpy(), resid=resid,
                   score=score, age=self._age)
        self.t += 1
        return out

    def _slow_step(self, e, out):
        """Detector lento (CUSUM) sul residuo e (2,). Prima l'eventuale
        riferimento di sessione, poi accumula."""
        c = self.cfg
        z2 = np.minimum((e / self.cusum_sigma) ** 2, c.cusum_clip)
        if self.ref is None:
            if self.state == NORMALE:              # riferimento solo in condizioni quiete
                self._ref_sum += z2
                self._ref_n += 1
                if self._ref_n >= self.nRef:
                    self.ref = np.maximum(self._ref_sum / self._ref_n, 1e-3)
            return
        # g limitato a 1.2 h: dopo la fine della perturbazione l'allarme lento si
        # chiude in pochi secondi (scende di ~1+kappa-z2 a campione), cosi' non
        # trasforma in "persistente" una perturbazione breve gia' finita.
        cap = np.where(np.isfinite(self.h), 1.2 * self.h, np.inf)
        self.g = np.minimum(np.maximum(0.0, self.g + z2 / self.ref - 1.0 - c.cusum_kappa), cap)
        hz = c.hz
        for ci, ch in enumerate(CHANNELS):
            ev = self._slow_ev[ci]
            if ev is None and self.g[ci] > self.h[ci]:
                # se l'altro canale e' gia' in allarme, questo e' probabilmente un
                # effetto indiretto (la storia perturbata sporca la predizione)
                other = [CHANNELS[k] for k in range(len(CHANNELS))
                         if k != ci and self._slow_ev[k] is not None]
                ev = {"canale": ch, "t_alarm": self.t, "t_alarm_s": self.t / hz,
                      "classe": "persistente", "inizio": "graduale",
                      "secondario": bool(other), "origine": other[0] if other else ch,
                      "ref": float(self.ref[ci])}
                self._slow_ev[ci] = ev
                self.slow_events.append(ev)
                out["slow_alarm"].append(ch)
            elif ev is not None:
                pend = self._pending_end[ci]
                if self.g[ci] < c.cusum_off * self.h[ci]:
                    if pend is None:
                        self._pending_end[ci] = self.t
                    elif self.t - pend >= self.nMerge:      # sotto abbastanza a lungo: fine
                        ev["t_end_s"] = pend / hz
                        ev["durata_s"] = (pend - ev["t_alarm"]) / hz
                        self._slow_ev[ci] = None
                        self._pending_end[ci] = None
                        out["slow_cleared"].append(ch)
                elif pend is not None and self.g[ci] > self.h[ci]:
                    self._pending_end[ci] = None            # ripreso: stesso evento
        out["g"] = self.g.copy()
        out["slow"] = np.array([e is not None for e in self._slow_ev])

    def _slow_active(self):
        """(2,) bool: allarme lento attivo per canale."""
        return np.array([e is not None for e in self._slow_ev])

    def _rise(self, ci):
        """Crescita di S_R/soglia del canale ci negli ultimi Tb secondi
        (valore attuale meno il valore di Tb fa, o il piu' vecchio disponibile)."""
        past = [float(r[ci]) for r in self._ratioR if np.isfinite(r[ci])]
        return past[-1] - past[0] if len(past) > 1 else 0.0

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
            rise = None if cls == "nessuna" else round(float(ev["rise"][ci]), 3)
            chans[c] = {"classe": cls, "inizio": onset, "salita_SR": rise}
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