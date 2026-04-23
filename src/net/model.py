import torch
import torch.nn as nn


class FishSensorEstimator(nn.Module):
    def __init__(self, input_size=1, gru_hidden=64, mlp_hidden=128):
        """
        Stimatore: data una finestra temporale di comandi motore,
        predice la risposta sensoriale attesa.

        input_size:  numero di canali in ingresso (1 => solo tail_target_rad)
        gru_hidden:  dimensione hidden state GRU
        mlp_hidden:  dimensione hidden layer MLP    
        """
        super().__init__()

        # Stadio 1: GRU - encoder temporale 
        # input:  (batch, window, 1)  =>  storia dei comandi motore
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

        # testa [sensor_diff, sensor_mean, v_flow]  =>  (3,)
        self.head = nn.Linear(mlp_hidden // 2, 3)

    def forward(self, seq):
        """
        seq:     (batch, window, 1)  =>  storia comandi motore normalizzati
        returns:
            pred  (batch, 3)  =>  [sensor_diff, sensor_mean, v_flow] normalizzati
            h     (batch, gru_hidden)  =>  contesto temporale
        """
        _, h = self.gru(seq)
        h = h.squeeze(0)   # (batch, gru_hidden)

        x = self.mlp(h)
        pred = self.head(x)   # (batch, 3)

        return pred, h