import ast
import numpy as np
import pandas as pd
import torch
from torch.utils.data import Dataset, Subset
from pathlib import Path
from sklearn.preprocessing import StandardScaler



#USO 
# python3 src/net/Estimator/dataset.py <DATASET PATH> <SCALER PATH> 

# H: lunghezza della finestra di storia in ingresso (H comandi + H sensori passati).
# A 10 Hz di logging effettivo, 20 timestep = 2 s = un ciclo completo a 0.5 Hz
# (freq minima), cosi' la finestra contiene sempre almeno un'oscillazione intera.
H = 20

# P: orizzonte di predizione. Le teste (IM e FM) sputano P passi in un colpo solo.
# DEVE ESSERE UGUALE A P IN MODEL
P = 10

NEEDED_COLS = ["present_current_ma", "tail_target_rad", "tail_amp_rad",
               "tail_freq_hz", "center_rad", "t_rel_sec"]

# canali in ingresso alle DUE reti (entrambe ricevono lo stesso ingresso):
#   comando:  [cmd]                       -> 1 canale  (C_T)
#   sensori:  [sensor_diff, current]      -> 2 canali  (S_T)
N_CMD_CHANNELS  = 1
N_SENS_CHANNELS = 2

# CTX_DIM: contesto statico [amp, freq, center, dt] iniettato nell'MLP di
# ENTRAMBE le reti (non nella GRU).
#   center = tail_bias_rad + tail_bias_rand_rad  (centro di oscillazione;
#            il feedback sensoriale bias_offset e' ignorato: spento in training)
#   dt     = diff(t_rel_sec)  (jitter reale dei timestep ROS)
CTX_DIM = 4


class FishJointDataset(Dataset):
    """Dataset unico per l'addestramento congiunto di IM (inversa) e FM (diretta).

    Ogni finestra restituisce TUTTO cio' che serve a entrambe le reti:
      seq_cmd   (H, 1)   storia comandi         -> C_T
      seq_sens  (H, 2)   storia sensori         -> S_T
      ctx       (4,)     contesto statico [amp, freq, center, dt]
      tgt_cmd   (P, 1)   comandi futuri (target IM)
      tgt_sens  (P, 2)   sensori futuri (target FM)
      label     (2,)     [amp_des, freq_des] fisici, ancorati a t=i-1

    Lo scaler e' UNICO e condiviso (chiavi: sd, vf, cmd, amp, freq, center, dt):
    cosi' le due
    reti normalizzano gli stessi segnali allo stesso modo, requisito per chiudere
    il ciclo in Fase B. Fit solo sui trial di train (niente leak).
    """

    def __init__(self, log_dir: str, h: int = H, p: int = P, scaler_path: str = None):
        self.h = h
        self.p = p

        self.seq_cmd     = []
        self.seq_sens    = []
        self.context     = []
        self.tgt_cmd     = []
        self.tgt_sens    = []
        self.labels      = []
        self.window_trial = []
        self.trial_names  = []

        self.norm_stats = {}
        self.scalers    = {}

        csv_files = list(Path(log_dir).glob("trial_*.csv"))
        if not csv_files:
            raise FileNotFoundError(f"Nessun csv in {log_dir}")
        print(f"Trovati {len(csv_files)} trial.")

        episodes = []
        for csv_path in csv_files:
            try:
                df = pd.read_csv(csv_path)
                ep = self._extract_signals(df, h, p, csv_path.name)
                ep["name"] = csv_path.name
                episodes.append(ep)
            except Exception as e:
                print(f"  Skipped {csv_path.name}: {e}")
        if not episodes:
            raise ValueError("Nessun episodio valido dopo il parsing dei CSV.")

        self._episodes    = episodes
        self._scaler_path  = scaler_path
        self.trial_names   = [ep["name"] for ep in episodes]
        self._prepared     = False
        self._prepare_lock = __import__("threading").Lock()
        self._device       = None

        print(f"Dataset grezzo: {len(episodes)} trial letti. "
              f"Scaler e finestre costruiti in split_by_trial() (fit solo su train).")

    # ---------- estrazione segnali (identica alla tua pipeline) ----------

    def _parse_sensor_values(self, series: pd.Series):
        def parse_one(s):
            try:
                vals = ast.literal_eval(str(s))
                if len(vals) < 2:
                    return [0.0, 0.0]
                out = [float(v) for v in vals[:2]]
                if not all(np.isfinite(out)):
                    return [np.nan, np.nan]
                return out
            except Exception:
                return [np.nan, np.nan]
        parsed = series.apply(parse_one)
        return np.array(parsed.tolist(), dtype=np.float32)

    def _extract_signals(self, df: pd.DataFrame, h: int, p: int, fname: str = ""):
        if len(df) < h + p + 1:
            raise ValueError(f"impossibile costruire finestre ({len(df)} righe < {h + p + 1})")
        if len(df) < 50:
            print(f"  [{fname}] attenzione: calibrazione a riposo su {len(df)} campioni (< 50)")

        df = df.copy()
        for c in NEEDED_COLS:
            if c not in df.columns:
                raise ValueError(f"colonna mancante: {c}")
            df[c] = pd.to_numeric(df[c], errors="coerce")

        n_nan = int(df[NEEDED_COLS].isna().sum().sum())
        if n_nan:
            frac = n_nan / (len(df) * len(NEEDED_COLS))
            if frac > 0.05:
                raise ValueError(f"troppi NaN ({n_nan}, {frac:.1%})")
            print(f"  [{fname}] {n_nan} NaN interpolati")
            df[NEEDED_COLS] = df[NEEDED_COLS].interpolate(limit_direction="both")

        sensors = self._parse_sensor_values(df["sensor_values"])
        if np.isnan(sensors).any():
            n_bad = int(np.isnan(sensors[:, 0]).sum())
            if n_bad / len(sensors) > 0.05:
                raise ValueError(f"troppe righe sensor_values invalide ({n_bad})")
            for k in range(sensors.shape[1]):
                col = sensors[:, k]
                mask = np.isnan(col)
                col[mask] = np.interp(np.flatnonzero(mask),
                                      np.flatnonzero(~mask), col[~mask])
                sensors[:, k] = col

        sensor_diff = sensors[:, 0] - sensors[:, 1]
        sensor_mean = (sensors[:, 0] + sensors[:, 1]) / 2.0
        offset      = sensor_diff[:50].mean()
        offset_mean = sensor_mean[:50].mean()
        sensor_diff_cal = sensor_diff - offset
        sensor_mean_cal = sensor_mean - offset_mean

        current   = df["present_current_ma"].values.astype(np.float32)
        cmd_servo = df["tail_target_rad"].values.astype(np.float32)
        amp_des   = df["tail_amp_rad"].values.astype(np.float32)
        freq_des  = df["tail_freq_hz"].values.astype(np.float32)

        # center = centro di oscillazione EFFETTIVO, gia' collassato nel log
        # (bias base + variazioni rand/turning). Il feedback sensoriale non c'e':
        # e' solo diagnostica, spento in training/test.
        center = df["center_rad"].values.astype(np.float32)

        # dt = jitter reale dei timestep ROS, ricavato da t_rel (non c'e' colonna dt).
        # Il primo campione non ha un dt a monte: replico il secondo valore.
        t_rel = df["t_rel_sec"].values.astype(np.float32)
        dt = np.diff(t_rel, prepend=t_rel[0]).astype(np.float32)
        if len(dt) > 1:
            dt[0] = dt[1]

        return {
            "sensor_diff_cal": sensor_diff_cal.astype(np.float32),
            "sensor_mean_cal": sensor_mean_cal.astype(np.float32),
            "cmd_servo":       cmd_servo,
            "current":         current,
            "amp_des":         amp_des,
            "freq_des":        freq_des,
            "center":          center,
            "dt":              dt,
            "offset_diff":     float(offset),
            "offset_mean":     float(offset_mean),
        }

    # ---------- scaler unico condiviso ----------

    def _fit_scalers(self, episodes):
        all_sd     = np.concatenate([e["sensor_diff_cal"] for e in episodes]).reshape(-1, 1)
        all_cmd    = np.concatenate([e["cmd_servo"]       for e in episodes]).reshape(-1, 1)
        all_vf     = np.concatenate([e["current"]         for e in episodes]).reshape(-1, 1)
        all_amp    = np.concatenate([e["amp_des"]         for e in episodes]).reshape(-1, 1)
        all_freq   = np.concatenate([e["freq_des"]        for e in episodes]).reshape(-1, 1)
        all_center = np.concatenate([e["center"]          for e in episodes]).reshape(-1, 1)
        all_dt     = np.concatenate([e["dt"]              for e in episodes]).reshape(-1, 1)

        self.scalers = {
            "sd":     StandardScaler().fit(all_sd),
            "cmd":    StandardScaler().fit(all_cmd),
            "vf":     StandardScaler().fit(all_vf),
            "amp":    StandardScaler().fit(all_amp),
            "freq":   StandardScaler().fit(all_freq),
            "center": StandardScaler().fit(all_center),
            "dt":     StandardScaler().fit(all_dt),
        }
        for sc in self.scalers.values():
            sc.scale_ = np.maximum(sc.scale_, 1e-3)
        self._sync_norm_stats()

    def _sync_norm_stats(self):
        self.norm_stats = {
            "sd_mean":     float(self.scalers["sd"].mean_[0]),
            "sd_std":      float(self.scalers["sd"].scale_[0]),
            "cmd_mean":    float(self.scalers["cmd"].mean_[0]),
            "cmd_std":     float(self.scalers["cmd"].scale_[0]),
            "vf_mean":     float(self.scalers["vf"].mean_[0]),
            "vf_std":      float(self.scalers["vf"].scale_[0]),
            "amp_mean":    float(self.scalers["amp"].mean_[0]),
            "amp_std":     float(self.scalers["amp"].scale_[0]),
            "freq_mean":   float(self.scalers["freq"].mean_[0]),
            "freq_std":    float(self.scalers["freq"].scale_[0]),
            "center_mean": float(self.scalers["center"].mean_[0]),
            "center_std":  float(self.scalers["center"].scale_[0]),
            "dt_mean":     float(self.scalers["dt"].mean_[0]),
            "dt_std":      float(self.scalers["dt"].scale_[0]),
        }

    # ---------- costruzione finestre ----------

    def _build_windows(self, ep, h, p, trial_idx):
        sc = self.scalers
        sd_n     = sc["sd"].transform(ep["sensor_diff_cal"].reshape(-1, 1)).ravel()
        cmd_n    = sc["cmd"].transform(ep["cmd_servo"].reshape(-1, 1)).ravel()
        vf_n     = sc["vf"].transform(ep["current"].reshape(-1, 1)).ravel()
        amp_n    = sc["amp"].transform(ep["amp_des"].reshape(-1, 1)).ravel()
        freq_n   = sc["freq"].transform(ep["freq_des"].reshape(-1, 1)).ravel()
        center_n = sc["center"].transform(ep["center"].reshape(-1, 1)).ravel()
        dt_n     = sc["dt"].transform(ep["dt"].reshape(-1, 1)).ravel()

        amp_des  = ep["amp_des"]
        freq_des = ep["freq_des"]
        n = len(cmd_n)

        # input: storia [0:H] a t=i-1 ; target: finestra futura [i : i+P]
        # range(h, n - p + 1): con l'ultima i, il target ...[i:i+p] esiste.
        for i in range(h, n - p + 1):
            seq_cmd  = cmd_n[i - h:i].reshape(-1, 1)                 # (H, 1)
            seq_sens = np.stack([sd_n[i - h:i], vf_n[i - h:i]], axis=1)  # (H, 2)

            tgt_cmd  = cmd_n[i:i + p].reshape(-1, 1)                 # (P, 1)
            tgt_sens = np.stack([sd_n[i:i + p], vf_n[i:i + p]], axis=1)  # (P, 2)

            # contesto statico ancorato a t=i-1 (ultimo istante di input):
            # [amp, freq, center, dt], tutti normalizzati.
            ctx = np.array([
                amp_n[i - 1],
                freq_n[i - 1],
                center_n[i - 1],
                dt_n[i - 1],
            ], dtype=np.float32)

            label = np.array([amp_des[i - 1], freq_des[i - 1]], dtype=np.float32)

            self.seq_cmd.append(seq_cmd)
            self.seq_sens.append(seq_sens)
            self.context.append(ctx)
            self.tgt_cmd.append(tgt_cmd)
            self.tgt_sens.append(tgt_sens)
            self.labels.append(label)
            self.window_trial.append(trial_idx)

    def _finalize_windows(self):
        self.seq_cmd  = torch.tensor(np.array(self.seq_cmd),  dtype=torch.float32)
        self.seq_sens = torch.tensor(np.array(self.seq_sens), dtype=torch.float32)
        self.context  = torch.tensor(np.array(self.context),  dtype=torch.float32)
        self.tgt_cmd  = torch.tensor(np.array(self.tgt_cmd),  dtype=torch.float32)
        self.tgt_sens = torch.tensor(np.array(self.tgt_sens), dtype=torch.float32)
        self.labels   = torch.tensor(np.array(self.labels),   dtype=torch.float32)
        self.window_trial = np.asarray(self.window_trial, dtype=np.int64)

        for name, t in [("seq_cmd", self.seq_cmd), ("seq_sens", self.seq_sens),
                        ("context", self.context), ("tgt_cmd", self.tgt_cmd),
                        ("tgt_sens", self.tgt_sens)]:
            if not torch.isfinite(t).all():
                raise ValueError(f"NaN/Inf residui in {name}: controlla i CSV con check_nan.py")

        if self._device is not None:
            self._move_tensors(self._device)

        print(f"Dataset: {len(self)} campioni da {len(self._episodes)} trial. "
              f"H={self.h} P={self.p} | seq_cmd={tuple(self.seq_cmd.shape[1:])} "
              f"seq_sens={tuple(self.seq_sens.shape[1:])}")

    # ---------- prepare / split (leak-free, come i tuoi) ----------

    def prepare(self, train_trial_ids):
        if self._prepared:
            return
        with self._prepare_lock:
            if self._prepared:
                return
            self._prepare_locked(train_trial_ids)

    def _prepare_locked(self, train_trial_ids):
        train_trial_ids = set(int(i) for i in train_trial_ids)
        train_episodes  = [ep for idx, ep in enumerate(self._episodes)
                           if idx in train_trial_ids]
        if not train_episodes:
            raise ValueError("prepare(): nessun trial di train per fittare lo scaler.")

        scaler_path = self._scaler_path
        if scaler_path is not None and Path(scaler_path).exists():
            self.scalers = self.load_scalers(scaler_path)
            needed = {"sd", "vf", "cmd", "amp", "freq", "center", "dt"}
            missing = [k for k in needed if k not in self.scalers]
            if missing:
                raise ValueError(
                    f"Lo scalers.pkl in {scaler_path} non contiene {missing}: "
                    f"incompatibile con il dataset congiunto. Cancellalo o usa "
                    f"un nuovo --scaler_path per rifittarlo."
                )
            self._sync_norm_stats()
            print(f"Normalizzatore caricato da {scaler_path} (nessun refit).")
        else:
            self._fit_scalers(train_episodes)
            if scaler_path is not None:
                self.save_scalers(scaler_path)
                print(f"Normalizzatore fittato SUL TRAIN e salvato in {scaler_path}.")
            else:
                print("Normalizzatore fittato SUL TRAIN (non salvato).")

        self.seq_cmd = []; self.seq_sens = []; self.context = []
        self.tgt_cmd = []; self.tgt_sens = []; self.labels = []; self.window_trial = []
        for trial_idx, ep in enumerate(self._episodes):
            self._build_windows(ep, self.h, self.p, trial_idx)

        self._finalize_windows()
        self._prepared = True

    def split_by_trial(self, val_frac=0.2, seed=42):
        trial_ids = np.arange(len(self._episodes))
        rng = np.random.default_rng(seed)
        rng.shuffle(trial_ids)

        n_val_trials = max(1, int(round(val_frac * len(trial_ids))))
        val_trials   = set(trial_ids[:n_val_trials].tolist())
        train_trials = [int(t) for t in trial_ids if t not in val_trials]
        if not train_trials or not val_trials:
            raise ValueError(
                f"Split per-trial degenere: train={len(train_trials)} trial, "
                f"val={len(val_trials)} trial. Servono piu' trial ({len(trial_ids)})."
            )

        self.prepare(train_trials)

        val_mask  = np.isin(self.window_trial, list(val_trials))
        val_idx   = np.flatnonzero(val_mask)
        train_idx = np.flatnonzero(~val_mask)
        if len(train_idx) == 0 or len(val_idx) == 0:
            raise ValueError(
                f"Split degenere a livello finestra: train={len(train_idx)}, val={len(val_idx)}.")

        return Subset(self, train_idx), Subset(self, val_idx)

    # ---------- device / io ----------

    def to(self, device):
        self._device = device
        if self._prepared:
            self._move_tensors(device)
        return self

    def _move_tensors(self, device):
        self.seq_cmd  = self.seq_cmd.to(device)
        self.seq_sens = self.seq_sens.to(device)
        self.context  = self.context.to(device)
        self.tgt_cmd  = self.tgt_cmd.to(device)
        self.tgt_sens = self.tgt_sens.to(device)
        self.labels   = self.labels.to(device)

    def save_scalers(self, path):
        import joblib
        joblib.dump(self.scalers, path)

    @staticmethod
    def load_scalers(path):
        import joblib
        return joblib.load(path)

    def __len__(self):
        return len(self.seq_cmd)

    def __getitem__(self, idx):
        return (
            self.seq_cmd[idx],
            self.seq_sens[idx],
            self.context[idx],
            self.tgt_cmd[idx],
            self.tgt_sens[idx],
            self.labels[idx],
        )


if __name__ == '__main__':
    import sys
    log_dir     = sys.argv[1] if len(sys.argv) > 1 else "src/net/dataset"
    scaler_path = sys.argv[2] if len(sys.argv) > 2 else str(Path(log_dir) / "scalers_joint.pkl")

    ds = FishJointDataset(log_dir, scaler_path=scaler_path)
    ds.split_by_trial(val_frac=0.2, seed=42)
    seq_cmd, seq_sens, ctx, tgt_cmd, tgt_sens, label = ds[0]
    print(f"seq_cmd:  {seq_cmd.shape}   (H, {N_CMD_CHANNELS})")
    print(f"seq_sens: {seq_sens.shape}   (H, {N_SENS_CHANNELS})")
    print(f"ctx:      {ctx.shape}   ({CTX_DIM},)")
    print(f"tgt_cmd:  {tgt_cmd.shape}   (P, {N_CMD_CHANNELS})")
    print(f"tgt_sens: {tgt_sens.shape}   (P, {N_SENS_CHANNELS})")
    print(f"label:    {label.shape}")
    print(f"norm_stats keys: {list(ds.norm_stats.keys())}")