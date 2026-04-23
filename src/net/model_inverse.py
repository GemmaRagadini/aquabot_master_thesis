import torch
import torch.nn as nn


class FishInverseEstimator(nn.Module):
    def __init__(self, input_size=3, gru_hidden=64, mlp_hidden=128):
        """
        Stimatore inverso: data una finestra temporale di letture sensoriali,
        predice il comando motore necessario per produrle.

        input_size:  numero di canali in ingresso (3 => sensor_diff, sensor_mean, v_flow)
        gru_hidden:  dimensione hidden state GRU
        mlp_hidden:  dimensione hidden layer MLP
        """
        super().__init__()

        # Stadio 1: encoder temporale
        # input:  (batch, window, 3)  =>  storia dei sensori
        # output: ultimo hidden state h(t) di dim gru_hidden
        self.gru = nn.GRU(
            input_size=input_size,
            hidden_size=gru_hidden,
            num_layers=1,
            batch_first=True,
        )

        # Stadio 2: MLP
        # input: h(t)  =>  dim gru_hidden
        self.mlp = nn.Sequential(
            nn.Linear(gru_hidden, mlp_hidden),
            nn.ReLU(),
            nn.Linear(mlp_hidden, mlp_hidden // 2),
            nn.ReLU(),
        )

        # Testa unica: predice tail_target_rad  =>  (1,)
        self.head = nn.Linear(mlp_hidden // 2, 1)

    def forward(self, seq):
        """
        seq:     (batch, window, 3)  =>  storia sensori normalizzati
                                         [sensor_diff, sensor_mean, v_flow]
        returns:
            cmd   (batch,)           =>  tail_target_rad normalizzato
            h     (batch, gru_hidden) =>  contesto temporale
        """
        _, h = self.gru(seq)
        h = h.squeeze(0)        # (batch, gru_hidden)

        x = self.mlp(h)
        cmd = self.head(x).squeeze(1)   # (batch,)

        return cmd, h