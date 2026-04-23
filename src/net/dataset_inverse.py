import ast
import numpy as np
import pandas as pd
import torch
from torch.utils.data import Dataset
from pathlib import Path

# a 20Hz, 30 timestep = 1.5 secondi
WINDOW    = 30
SAMPLE_HZ = 20.0


class FishInverseDataset(Dataset):
    def __init__(self, log_dir: str, window: int = WINDOW):
        self.sequences = []   # (window, 3)  storia sensori
        self.targets   = []   # (1,)         tail_target_rad

        self.norm_stats = {}

        csv_files = list(Path(log_dir).glob("trial_*.csv"))
        if not csv_files:
            raise FileNotFoundError(f"Nessun csv in {log_dir}")

        print(f"Trovati {len(csv_files)} trial.")

        for csv_path in csv_files:
            try:
                df = pd.read_csv(csv_path)
                self._process_episode(df, window)
            except Exception as e:
                print(f"  Skipped {csv_path.name}: {e}")

        self.sequences = torch.tensor(np.array(self.sequences), dtype=torch.float32)
        self.targets   = torch.tensor(np.array(self.targets),   dtype=torch.float32)

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

    def _process_episode(self, df: pd.DataFrame, window: int):
        # --- input: sensori ---
        sensors = self._parse_sensor_values(df["sensor_values"])
        sensor_diff = sensors[:, 0] - sensors[:, 1]
        sensor_mean = (sensors[:, 0] + sensors[:, 1]) / 2.0

        # calibrazione offset a riposo (primi 50 campioni)
        offset_diff = sensor_diff[:50].mean()
        offset_mean = sensor_mean[:50].mean()
        sensor_diff_cal = sensor_diff - offset_diff
        sensor_mean_cal = sensor_mean - offset_mean

        # v_flow: placeholder a zero finché non c'è il sensore
        v_flow = np.zeros(len(df), dtype=np.float32)

        # --- output: comando motore ---
        cmd = df["tail_target_rad"].values.astype(np.float32)

        # --- normalizzazione per-episodio ---
        sd_mean,  sd_std  = sensor_diff_cal.mean(), sensor_diff_cal.std() + 1e-6
        sm_mean,  sm_std  = sensor_mean_cal.mean(), sensor_mean_cal.std() + 1e-6
        vf_mean,  vf_std  = v_flow.mean(),          v_flow.std()          + 1e-6
        cmd_mean, cmd_std = cmd.mean(),              cmd.std()             + 1e-6

        sd_n  = (sensor_diff_cal - sd_mean)  / sd_std
        sm_n  = (sensor_mean_cal - sm_mean)  / sm_std
        vf_n  = (v_flow          - vf_mean)  / vf_std
        cmd_n = (cmd             - cmd_mean) / cmd_std

        self.norm_stats = {
            "sd_mean":  float(sd_mean),  "sd_std":  float(sd_std),
            "sm_mean":  float(sm_mean),  "sm_std":  float(sm_std),
            "vf_mean":  float(vf_mean),  "vf_std":  float(vf_std),
            "cmd_mean": float(cmd_mean), "cmd_std": float(cmd_std),
        }

        # --- finestre scorrevoli ---
        for i in range(window, len(df)):
            # input: finestra di sensori  =>  (window, 3)
            seq = np.stack([
                sd_n[i - window:i],
                sm_n[i - window:i],
                vf_n[i - window:i],
            ], axis=1)

            # target: comando motore al timestep corrente  =>  scalare
            target = np.array([cmd_n[i]], dtype=np.float32)

            self.sequences.append(seq)
            self.targets.append(target)

    def __len__(self):
        return len(self.sequences)

    def __getitem__(self, idx):
        return self.sequences[idx], self.targets[idx]


if __name__ == '__main__':
    import sys
    log_dir = sys.argv[1] if len(sys.argv) > 1 else "../../logs"
    ds = FishInverseDataset(log_dir)
    seq, target = ds[0]
    print(f"seq shape:    {seq.shape}")     # (30, 3)
    print(f"target shape: {target.shape}")  # (1,)
    print(f"norm_stats:   {ds.norm_stats}")