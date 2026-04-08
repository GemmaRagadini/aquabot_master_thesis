
import numpy as np
import pandas as pd
import torch
import ast
from torch.utils.data import Dataset
from pathlib import Path

# legge csv e trasforma in coppie (input, output) per il training

WINDOW    = 30    # significa che ogni campione dato alla rete contiene gli ultimi 30 timestep, cioè 30/20= 1.5 secondi di storia
SAMPLE_HZ = 20.0
 
 
class FishDataset(Dataset):
    def __init__(self, log_dir: str, window: int = WINDOW):
        self.sequences = []
        self.targets   = []
        self.labels    = []

        # statistiche per normalizzazione (salvate nel checkpoint)
        self.norm_stats = {}

        csv_files = list(Path(log_dir).glob("trial_*.csv"))
        if not csv_files: 
            raise FileNotFoundError(f"Nessun csv in {log_dir}") 
        print(f"Trovati {len(csv_files)} trial.")
        for csv_path in csv_files:
            try: 
                df=pd.read_csv(csv_path)
                self._process_episode(df, window) 
            except Exception as e: 
                print (f" Skipped {csv_path.name}: {e}")

        self.sequences = torch.tensor(np.array(self.sequences), dtype= torch.float32)
        self.targets = torch.tensor(np.array(self.targets), dtype=torch.float32)
        self.labels = torch.tensor(np.array(self.labels), dtype=torch.float32)

        print(f"Dataset: {len(self)} campioni da {len(csv_files)} trial.")

    def _parse_sensor_values(self, series: pd.Series): 
        # converte la colonna sensor_values da stringa [v1,v2] a array np 
        def parse_one(s): 
            try: 
                vals = ast.literal_eval(str(s)) 
                return [float(v) for v in vals] 
            except Exception: 
                return [0.0, 0.0]
        parsed = series.apply(parse_one)
        return np.array(parsed.tolist(), dtype=np.float32)
            
    # episode = trial = csv 
    def _process_episode(self, df: pd.DataFrame, window: int):
        # sensori 
        sensors = self._parse_sensor_values(df["sensor_values"]) 
        # sensor_diff : differenza tra i due bending 
        sensor_diff = sensors[:,0]-sensors[:,1]
        # offset a riposo : media dei primi 50 campioni => va bene ?? 
        offset = sensor_diff[:50].mean()
        sensor_diff_cal = sensor_diff- offset 

        #CAMBIARE QUANDO CI SARÀ IL SENSORE DI FLUSSO
        v_flow = np.zeros(len(df), dtype = np.float32)  
         
        cmd_servo=df["tail_target_rad"].values.astype(np.float32) 
        cmd_servo=df["tail_amp_rad"].values.astype(np.float32)
        cmd_servo=df["tail_freq_hz"].values.astype(np.float32)

        #normalizzazione per ogni csv
        sd_mean,  sd_std  = sensor_diff_cal.mean(), sensor_diff_cal.std() + 1e-6
        cmd_mean, cmd_std = cmd_servo.mean(),        cmd_servo.std()       + 1e-6
        vf_mean,  vf_std  = v_flow.mean(),           v_flow.std()          + 1e-6
 
        sd_n  = (sensor_diff_cal - sd_mean)  / sd_std
        cmd_n = (cmd_servo       - cmd_mean) / cmd_std
        vf_n  = (v_flow          - vf_mean)  / vf_std
 
        # salva le statistiche 
        self.norm_stats = {
            "sd_mean": float(sd_mean), "sd_std": float(sd_std),
            "cmd_mean": float(cmd_mean), "cmd_std":float(cmd_std), 
            "vf_mean": float(vf_mean), "vf_std": float(vf_std),
        }

        # finestre scorrevoli 
        for i in range(window, len(df)):  
            seq = np.stack([ 
                sd_n[i-window:i], 
                vf_n[i-window,i],
            ], axis = 1)
        
        # target : [amp_des, freq_des] al timestamp corrente 
        target = np.array([amp_des[i], freq_des[i]], dtype= np.float32)

        # label (ground truth da predire): [cmd_servo, v_flow] 
        label = np.array([cmd_n[i], vf_n[i]], dtype = np.float32) 
        self.sequences.append(seq) 
        self.targets.append(target) 
        self.labels.append(label)


    def __len__(self):
        return len(self.sequences)
 
    def __getitem__(self, idx):
        return self.sequences[idx], self.targets[idx], self.labels[idx]
    # prosegui da qua 
    if __name__=='__main__':