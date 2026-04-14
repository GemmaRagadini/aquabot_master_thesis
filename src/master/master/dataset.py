import ast
import numpy as np
import pandas as pd
import torch
from torch.utils.data import Dataset
from pathlib import Path

# WINDOW: quanti timestep di storia diamo alla GRU
# a 20Hz, 30 timestep = 1.5 secondi
WINDOW    = 30
SAMPLE_HZ = 20.0


class FishDataset(Dataset):
    def __init__(self, log_dir: str, window: int = WINDOW):
        self.sequences = []
        self.targets   = []
        self.labels    = []

        # statistiche di normalizzazione (salvate nel checkpoint per runtime)
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
        self.labels    = torch.tensor(np.array(self.labels),    dtype=torch.float32)

        print(f"Dataset: {len(self)} campioni da {len(csv_files)} trial.")

    def _parse_sensor_values(self, series: pd.Series):
        # converte la colonna sensor_values da stringa "[v1, v2]" a array numpy
        def parse_one(s):
            try:
                vals = ast.literal_eval(str(s))
                if len(vals) < 2:       # lista vuota o incompleta
                    return [0.0, 0.0]
                return [float(v) for v in vals]
            except Exception:
                return [0.0, 0.0]

        parsed = series.apply(parse_one)
        return np.array(parsed.tolist(), dtype=np.float32)

    def _process_episode(self, df: pd.DataFrame, window: int):
        # --- sensori ---
        sensors = self._parse_sensor_values(df["sensor_values"])
        sensor_diff = sensors[:, 0] - sensors[:, 1]
        sensor_mean = (sensors[:, 0] + sensors[:, 1]) / 2.0   

        # offset a riposo: media dei primi 50 campioni (come fa master_node)
        offset = sensor_diff[:50].mean()
        sensor_diff_cal = sensor_diff - offset

        # offset a riposo anche per la media
        offset_mean = sensor_mean[:50].mean()
        sensor_mean_cal = sensor_mean - offset_mean

        # --- v_flow: placeholder a zero finché non c'è il sensore ---
        v_flow = np.zeros(len(df), dtype=np.float32)
        # v_flow = df["v_flow"].values.astype(np.float32)  # decommentare quando disponibile

        # --- ground truth ---
        cmd_servo = df["tail_target_rad"].values.astype(np.float32)
        amp_des   = df["tail_amp_rad"].values.astype(np.float32)
        freq_des  = df["tail_freq_hz"].values.astype(np.float32)

        # --- normalizzazione per-episodio ---
        sd_mean,  sd_std  = sensor_diff_cal.mean(), sensor_diff_cal.std() + 1e-6
        sm_mean,  sm_std  = sensor_mean_cal.mean(), sensor_mean_cal.std() + 1e-6
        cmd_mean, cmd_std = cmd_servo.mean(),        cmd_servo.std()       + 1e-6
        vf_mean,  vf_std  = v_flow.mean(),           v_flow.std()          + 1e-6

        sd_n  = (sensor_diff_cal - sd_mean) / sd_std
        sm_n  = (sensor_mean_cal - sm_mean) / sm_std
        cmd_n = (cmd_servo       - cmd_mean) / cmd_std
        vf_n  = (v_flow          - vf_mean)  / vf_std

        # salva le statistiche dell'episodio
        self.norm_stats = {
            "sd_mean":  float(sd_mean),  "sd_std":  float(sd_std),
            "sm_mean":  float(sm_mean),  "sm_std":  float(sm_std),
            "cmd_mean": float(cmd_mean), "cmd_std": float(cmd_std),
            "vf_mean":  float(vf_mean),  "vf_std":  float(vf_std),
        }

        # --- finestre scorrevoli ---
        for i in range(window, len(df)):
            seq = np.stack([
                sd_n[i - window:i],   # sensor_diff  — deformazione laterale
                sm_n[i - window:i],   # sensor_mean  — deformazione assiale media
                vf_n[i - window:i],   # v_flow
            ], axis=1)                # shape: (window, 3)

            target = np.array([amp_des[i], freq_des[i]], dtype=np.float32)  # (2,)
            label  = np.array([cmd_n[i],   vf_n[i]],    dtype=np.float32)   # (2,)

            self.sequences.append(seq)
            self.targets.append(target)
            self.labels.append(label)

    def __len__(self):
        return len(self.sequences)

    def __getitem__(self, idx):
        return self.sequences[idx], self.targets[idx], self.labels[idx]


if __name__ == '__main__':
    import sys
    log_dir = sys.argv[1] if len(sys.argv) > 1 else "../../logs"
    ds = FishDataset(log_dir)
    seq, target, label = ds[0]
    print(f"seq shape:    {seq.shape}")     # (30, 3)
    print(f"target shape: {target.shape}")  # (2,)
    print(f"label shape:  {label.shape}")   # (2,)
    print(f"norm_stats:   {ds.norm_stats}")