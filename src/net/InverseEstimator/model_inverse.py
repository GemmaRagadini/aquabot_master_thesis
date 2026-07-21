import torch
import torch.nn as nn

# H: quanti istanti passati predice la testa storia
# a 20 Hz, 20 timestep = 1 secondo = un ciclo completo a 1 Hz
H = 20


class FishInverseEstimator(nn.Module):
    def __init__(self, input_size=3, gru_hidden=64, mlp_hidden=128, h=H):
        """
        Stimatore inverso: data una finestra temporale di valori sensoriali,
        predice i comandi motore attesi.

        input_size:  numero di canali in ingresso (3 => sensor_diff, sensor_mean, current)
        gru_hidden:  dimensione hidden state GRU
        mlp_hidden:  dimensione hidden layer MLP
        h:       quanti istanti predice la testa storia (= lunghezza finestra input)
        """
        super().__init__()
        self.h = h

        # Stadio 1: GRU — encoder temporale
        # input:  (batch, h, 3)        => storia dei valori sensoriali
        # output: tutti gli hidden state   (batch, h, gru_hidden)
        #         + ultimo hidden state    (1, batch, gru_hidden)
        self.gru = nn.GRU(
            input_size=input_size,
            hidden_size=gru_hidden,
            num_layers=1,
            batch_first=True,
        )

        # Stadio 2: MLP — usato solo per la testa futuro
        # input: ultimo hidden state h(t)  (batch, gru_hidden)
        self.mlp = nn.Sequential(
            nn.Linear(gru_hidden, mlp_hidden),
            nn.ReLU(),
            nn.Linear(mlp_hidden, mlp_hidden // 2),
            nn.ReLU(),
        )

        # Testa storia: applicata su tutti gli h hidden state
        # (batch, h, gru_hidden) -> (batch, h, 1)
        # predice tail_target_rad per ogni istante passato
        self.head_history = nn.Linear(gru_hidden, 1)

        # Testa futuro: predice tail_target_rad al timestep t+1
        # (batch, mlp_hidden//2) -> (batch, 1)
        self.head_future = nn.Linear(mlp_hidden // 2, 1)

    def forward(self, seq):
        """
        seq:     (batch, h, 3)   => storia sensoriale normalizzata
                                        [sensor_diff, sensor_mean, current]

        returns:
            pred_history  (batch, h, 1)   => comandi agli ultimi h istanti passati
            pred_future   (batch, 1)           => comando al timestep t+1
            h             (batch, gru_hidden)  => contesto temporale
        """
        # all_h: (batch, h, gru_hidden) — tutti gli hidden state
        # h_n:   (1, batch, gru_hidden)     — solo l'ultimo
        all_h, h_n = self.gru(seq)
        h = h_n.squeeze(0)   # (batch, gru_hidden)

        # testa storia: tutti gli h hidden state -> comandi passati
        pred_history = self.head_history(all_h)          # (batch, h, 1)

        # testa futuro: ultimo hidden state -> MLP -> comando t+1
        x = self.mlp(h)
        pred_future = self.head_future(x)                # (batch, 1)

        return pred_history, pred_future, h