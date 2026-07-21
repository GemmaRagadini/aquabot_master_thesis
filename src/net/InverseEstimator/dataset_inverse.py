import ast
import numpy as np
import pandas as pd
import torch
from torch.utils.data import Dataset
from pathlib import Path

# H: quanti istanti passati sono nel target storia
# a 20Hz, 20 timestep = 1 secondo = un ciclo completo a 1 Hz
H = 20


class FishInverseDataset(Dataset):
    def __init__(self, log_dir: str, h: int = H):
        self.sequences       = []   # (h, 3)  storia sensoriale in ingresso
        self.targets_history = []   # (h, 1)  comandi passati
        self.targets_future  = []   # (1,)        comando al t+1
        self.labels          = []

        self.norm_stats = {}
        self.h = h

        csv_files = list(Path(log_dir).glob("trial_*.csv"))
        if not csv_files:
            raise FileNotFoundError(f"Nessun csv in {log_dir}")

        print(f"Trovati {len(csv_files)} trial.")

        for csv_path in csv_files:
            try:
                df = pd.read_csv(csv_path)
                self._process_episode(df, h)
            except Exception as e:
                print(f"  Skipped {csv_path.name}: {e}")

        self.sequences       = torch.tensor(np.array(self.sequences),       dtype=torch.float32)
        self.targets_history = torch.tensor(np.array(self.targets_history), dtype=torch.float32)
        self.targets_future  = torch.tensor(np.array(self.targets_future),  dtype=torch.float32)
        self.labels          = torch.tensor(np.array(self.labels),          dtype=torch.float32)

        print(f"Dataset: {len(self)} campioni da {len(csv_files)} trial.")

    def _parse_sensor_values(self, series: pd.Series):
        def parse_one(s):
            try:
                vals = ast.literal_eval(str(s))
                if len(vals) < 2:
                    return [0.0, 0.0]
                return [float(v) for v in vals]
            except Exception:
                return [0.0, 0.0]

        parsed = series.apply(parse_one)
        return np.array(parsed.tolist(), dtype=np.float32)

    def _process_episode(self, df: pd.DataFrame, h: int):
        # --- sensori ---
        sensors = self._parse_sensor_values(df["sensor_values"])
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

        # --- normalizzazione per-episodio ---
        sd_mean,  sd_std  = sensor_diff_cal.mean(), sensor_diff_cal.std() + 1e-6
        sm_mean,  sm_std  = sensor_mean_cal.mean(), sensor_mean_cal.std() + 1e-6
        cmd_mean, cmd_std = cmd_servo.mean(),        cmd_servo.std()       + 1e-6
        vf_mean,  vf_std  = current.mean(),           current.std()          + 1e-6

        sd_n  = (sensor_diff_cal - sd_mean) / sd_std
        sm_n  = (sensor_mean_cal - sm_mean) / sm_std
        cmd_n = (cmd_servo       - cmd_mean) / cmd_std
        vf_n  = (current          - vf_mean)  / vf_std

        self.norm_stats = {
            "sd_mean":  float(sd_mean),  "sd_std":  float(sd_std),
            "sm_mean":  float(sm_mean),  "sm_std":  float(sm_std),
            "cmd_mean": float(cmd_mean), "cmd_std": float(cmd_std),
            "vf_mean":  float(vf_mean),  "vf_std":  float(vf_std),
        }

        # --- finestre scorrevoli ---
        # il loop finisce a len(df)-1 perché serve il campione t+1 per il target futuro
        for i in range(h, len(df) - 1):
            # input: storia degli ultimi h valori sensoriali  →  (h, 3)
            seq = np.stack([
                sd_n[i - h:i],
                sm_n[i - h:i],
                vf_n[i - h:i],
            ], axis=1)

            # target storia: ultimi h comandi motore  →  (h, 1)
            target_history = cmd_n[i - h:i].reshape(-1, 1)

            # target futuro: comando al timestep t+1  →  (1,)
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
    log_dir = sys.argv[1] if len(sys.argv) > 1 else "../../logs/ds"
    ds = FishInverseDataset(log_dir)
    seq, t_hist, t_fut, label = ds[0]
    print(f"seq shape:            {seq.shape}")      # (20, 3)
    print(f"target_history shape: {t_hist.shape}")   # (20, 1)
    print(f"target_future shape:  {t_fut.shape}")    # (1,)
    print(f"label shape:          {label.shape}")    # (2,)
    print(f"norm_stats:           {ds.norm_stats}")
    print("\n--- 5 samples ---")
    for i in range(5):
        seq, t_hist, t_fut, label = ds[i]
        print(f"\n[Sample {i}]")
        print(f"  INPUT  seq (h,3)  -> shape {seq.shape}  | first: {seq[0].numpy()}")
        print(f"  TARGET hist (h,1) -> shape {t_hist.shape} | first: {t_hist[0].numpy()}")
        print(f"  TARGET fut  (1,)      -> shape {t_fut.shape}  | value: {t_fut.numpy()}")
        print(f"  LABEL  (amp,freq)     -> {label.numpy()}")