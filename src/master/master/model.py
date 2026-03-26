import torch
import torch.nn as nn
 
 
class FishStaticNet(nn.Module):
    def __init__(self, input_size=2, gru_hidden=64, mlp_hidden=128):
        super().__init__()
 
        # Stadio 1: encoder temporale
        self.gru = None          # TODO: nn.GRU(...)
 
        # Stadio 2: MLP
        self.mlp = None          # TODO: nn.Sequential(...)
 
        # Testa 1: comando servo
        self.head_cmd = None     # TODO: nn.Linear(...)
 
        # Testa 2: v_flow predetto
        self.head_vflow = None   # TODO: nn.Linear(...)
 
    def forward(self, seq, target):
        # seq:    (batch, window, input_size)
        # target: (batch, 2)  -> [amp_des, freq_des]
        # returns: cmd (batch,), vflow (batch,), h (batch, gru_hidden)
        pass