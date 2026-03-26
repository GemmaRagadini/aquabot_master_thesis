
import numpy as np
import pandas as pd
import torch
from torch.utils.data import Dataset
from pathlib import Path
 
WINDOW    = 30    # timestep — a 20Hz = 1.5s
SAMPLE_HZ = 20.0
 
 
class FishDataset(Dataset):
    def __init__(self, log_dir: str, window: int = WINDOW):
        self.sequences = []
        self.targets   = []
        self.labels    = []
 
        # TODO: iterare i CSV in log_dir e chiamare _process_episode
 
    def _process_episode(self, df: pd.DataFrame, window: int):
        # TODO:
        # 1. estrarre sensor_diff calibrato da sensor_values
        # 2. estrarre v_flow (placeholder 0 finché non c'è il sensore)
        # 3. normalizzare per-episodio
        # 4. costruire finestre scorrevoli (seq, target, label)
        pass
 
    def __len__(self):
        return len(self.sequences)
 
    def __getitem__(self, idx):
        return self.sequences[idx], self.targets[idx], self.labels[idx]