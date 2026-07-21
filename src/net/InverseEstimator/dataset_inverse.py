import ast
import numpy as np
import pandas as pd
import torch
from torch.utils.data import Dataset
from pathlib import Path

# H: quanti istanti passati sono nel target storia
# a 20Hz, 20 timestep = 1 secondo = un ciclo completo a 1 Hz
H = 20

NEEDED_COLS = ["present_current_ma", "tail_target_rad", "tail_amp_rad", "tail_freq_hz"]


class FishInverseDataset(Dataset):
    def __init__(self, dataset_dir: str, h: int = H):
        self.sequences       = []   # (h, 3)  storia sensoriale in ingresso
        self.targets_history = []   # (h, 1)  comandi passati
        self.targets_future  = []   # (1,)    comando al t+1
        self.labels          = []

        self.norm_stats = {}
        self.h = h

        csv_files = list(Path(dataset_dir).glob("trial_*.csv"))
        if not csv_files:
            raise FileNotFoundError(f"Nessun csv in {dataset_dir}")

        print(f"Trovati {len(csv_files)} trial.")

        for csv_path in csv_files:
            try:
                df = pd.read_csv(csv_path)
                self._process_episode(df, h, csv_path.name)
            except Exception as e:
                print(f"  Skipped {csv_path.name}: {e}")

        self.sequences       = torch.tensor(np.array(self.sequences),       dtype=torch.float32)
        self.targets_history = torch.tensor(np.array(self.targets_history), dtype=torch.float32)
        self.targets_future  = torch.tensor(np.array(self.targets_future),  dtype=torch.float32)
        self.labels          = torch.tensor(np.array(self.labels),          dtype=torch.float32)

        # --- guardia finale: nessun NaN/Inf deve arrivare al training ---
        for name, t in [("sequences", self.sequences),
                        ("targets_history", self.targets_history),
                        ("targets_future", self.targets_future)]:
            if not torch.isfinite(t).all():
                raise ValueError(f"NaN/Inf residui in {name}: controlla i CSV con check_nan.py")

        print(f"Dataset: {len(self)} campioni da {len(csv_files)} trial.")

    def to(self, device):
        """Sposta tutti i tensori sul device (es. GPU) una volta sola."""
        self.sequences       = self.sequences.to(device)
        self.targets_history = self.targets_history.to(device)
        self.targets_future  = self.targets_future.to(device)
        self.labels          = self.labels.to(device)
        return self

    def _parse_sensor_values(self, series: pd.Series):
        def parse_one(s):
            try:
                vals = ast.literal_eval(str(s))
                if len(vals) < 2:
                    return [0.0, 0.0]
                out = [float(v) for v in vals[:2]]
                if not all(np.isfinite(out)):
                    return [np.nan, np.nan]   # marcato, poi interpolato
                return out
            except Exception:
                return [np.nan, np.nan]

        parsed = series.apply(parse_one)
        return np.array(parsed.tolist(), dtype=np.float32)

    def _process_episode(self, df: pd.DataFrame, h: int, fname: str = ""):
        # nessun filtro sulla lunghezza: sotto h+2 righe non esiste però
        # alcuna finestra costruibile, quindi si salta solo quel caso limite
        if len(df) < h + 2:
            raise ValueError(f"impossibile costruire finestre ({len(df)} righe < {h + 2})")
        if len(df) < 50:
            print(f"  [{fname}] attenzione: calibrazione a riposo su {len(df)} campioni (< 50)")

        # --- coercizione numerica + interpolazione dei NaN ---
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

        # --- sensori (righe non parsabili -> NaN -> interpolazione) ---
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

        # offset a riposo: media dei primi 50 campioni (come fa master_node)
        offset      = sensor_diff[:50].mean()
        offset_mean = sensor_mean[:50].mean()
        sensor_diff_cal = sensor_diff - offset
        sensor_mean_cal = sensor_mean - offset_mean

        current   = df["present_current_ma"].values.astype(np.float32)
        cmd_servo = df["tail_target_rad"].values.astype(np.float32)

        # label invariata (amp, freq desiderati)
        amp_des  = df["tail_amp_rad"].values.astype(np.float32)
        freq_des = df["tail_freq_hz"].values.astype(np.float32)

        # --- normalizzazione per-episodio (std con floor, non solo +eps) ---
        def _norm(x):
            m, s = x.mean(), max(float(x.std()), 1e-3)
            return (x - m) / s, m, s

        sd_n,  sd_mean,  sd_std  = _norm(sensor_diff_cal)
        sm_n,  sm_mean,  sm_std  = _norm(sensor_mean_cal)
        cmd_n, cmd_mean, cmd_std = _norm(cmd_servo)
        vf_n,  vf_mean,  vf_std  = _norm(current)

        self.norm_stats = {
            "sd_mean":  float(sd_mean),  "sd_std":  float(sd_std),
            "sm_mean":  float(sm_mean),  "sm_std":  float(sm_std),
            "cmd_mean": float(cmd_mean), "cmd_std": float(cmd_std),
            "vf_mean":  float(vf_mean),  "vf_std":  float(vf_std),
        }

        # --- finestre scorrevoli ---
        # il loop finisce a len(df)-1 perché serve il campione t+1 per il target futuro
        for i in range(h, len(df) - 1):
            # input: storia degli ultimi h valori sensoriali  ->  (h, 3)
            seq = np.stack([
                sd_n[i - h:i],
                sm_n[i - h:i],
                vf_n[i - h:i],
            ], axis=1)

            # target storia: ultimi h comandi motore  ->  (h, 1)
            target_history = cmd_n[i - h:i].reshape(-1, 1)

            # target futuro: comando al timestep t+1  ->  (1,)
            target_future = np.array([cmd_n[i + 1]], dtype=np.float32)

            # label invariata (amp, freq desiderati)
            label = np.array([amp_des[i], freq_des[i]], dtype=np.float32)

            self.sequences.append(seq)
            self.targets_history.append(target_history)
            self.targets_future.append(target_future)
            self.labels.append(label)

    def __len__(self):
        return len(self.sequences)

    def __getitem__(self, idx):
        return (
            self.sequences[idx],
            self.targets_history[idx],
            self.targets_future[idx],
            self.labels[idx],
        )


if __name__ == '__main__':
    import sys
    dataset_dir = sys.argv[1] if len(sys.argv) > 1 else "./src/net/dataset"
    ds = FishInverseDataset(dataset_dir)
    seq, t_hist, t_fut, label = ds[0]
    print(f"seq shape:            {seq.shape}")      # (20, 3)
    print(f"target_history shape: {t_hist.shape}")   # (20, 1)
    print(f"target_future shape:  {t_fut.shape}")    # (1,)
    print(f"label shape:          {label.shape}")    # (2,)
    print(f"norm_stats:           {ds.norm_stats}")