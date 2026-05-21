import torch
import torch.nn as nn

# H_OUT: quanti istanti passati predice la testa storia
# a 20 Hz, 20 timestep = 1 secondo = un ciclo completo a 1 Hz
H_OUT = 20


class FishSensorEstimator(nn.Module):
    def __init__(self, input_size=1, gru_hidden=64, mlp_hidden=128, h_out=H_OUT):
        """
        Stimatore: data una finestra temporale di comandi motore,
        predice la risposta sensoriale attesa.

        input_size:  numero di canali in ingresso (1 => tail_target_rad normalizzato)
        gru_hidden:  dimensione hidden state GRU
        mlp_hidden:  dimensione hidden layer MLP
        h_out:       quanti istanti predice la testa storia (= lunghezza finestra input)
        """
        super().__init__()
        self.h_out = h_out

        # Stadio 1: GRU — encoder temporale
        # input:  (batch, h_out, 1)        => storia dei comandi motore (tail_target_rad)
        # output: tutti gli hidden state   (batch, h_out, gru_hidden)
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

        # Testa storia: applicata su tutti gli h_out hidden state
        # (batch, h_out, gru_hidden) -> (batch, h_out, 3)
        # predice [sensor_diff, sensor_mean, current] per ogni istante passato
        self.head_history = nn.Linear(gru_hidden, 3)

        # Testa futuro: predice [sensor_diff, sensor_mean, current] al timestep t+1
        # (batch, mlp_hidden//2) -> (batch, 3)
        self.head_future = nn.Linear(mlp_hidden // 2, 3)

    def forward(self, seq):
        """
        seq:     (batch, h_out, 1)   => storia comandi motore normalizzati (tail_target_rad)

        returns:
            pred_history  (batch, h_out, 3)   => sensori agli ultimi h_out istanti passati
            pred_future   (batch, 3)           => sensori al timestep t+1
            h             (batch, gru_hidden)  => contesto temporale
        """
        # all_h: (batch, h_out, gru_hidden) — tutti gli hidden state
        # h_n:   (1, batch, gru_hidden)     — solo l'ultimo
        all_h, h_n = self.gru(seq)
        h = h_n.squeeze(0)   # (batch, gru_hidden)

        # testa storia: tutti gli h_out hidden state -> sensori passati
        pred_history = self.head_history(all_h)          # (batch, h_out, 3)

        # testa futuro: ultimo hidden state -> MLP -> sensori t+1
        x = self.mlp(h)
        pred_future = self.head_future(x)                # (batch, 3)

        return pred_history, pred_future, h