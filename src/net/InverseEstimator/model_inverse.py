import torch
import torch.nn as nn

# H: quanti istanti passati predice la testa storia
# a 20 Hz, 20 timestep = 1 secondo = un ciclo completo a 1 Hz
H = 20

# N_OUTPUTS: quanti canali comando predicono le teste.
# 1 = [tail_target_rad]  (comando servo)
# Simmetrico a N_OUTPUTS in model.py (rete diretta): tenerlo come costante di modulo
# permette di cambiare i canali di output in un punto solo, senza toccare il forward.
N_OUTPUTS = 1


class FishInverseEstimator(nn.Module):
    def __init__(self, input_size=2, gru_hidden=64, mlp_hidden=128, h=H):
        """
        Stimatore inverso: data una finestra temporale di valori sensoriali,
        predice i comandi motore attesi.

        input_size:  numero di canali in ingresso (2 => sensor_diff, current)
                     sensor_mean escluso, speculare all'output della diretta.
                     Il valore effettivo viene comunque letto dal dataset in
                     train_inverse.py, quindi questo default e' solo indicativo.
        gru_hidden:  dimensione hidden state GRU
        mlp_hidden:  dimensione hidden layer MLP
        h:           quanti istanti predice la testa storia (= lunghezza finestra input)
        """
        super().__init__()
        self.h = h

        # Stadio 1: GRU — encoder temporale
        # input:  (batch, h, input_size)   => storia dei valori sensoriali
        # output: tutti gli hidden state    (batch, h, gru_hidden)
        #         + ultimo hidden state     (1, batch, gru_hidden)
        self.gru = nn.GRU(
            input_size=input_size,
            hidden_size=gru_hidden,
            num_layers=1,
            batch_first=True,
        )

        # Stadio 2a: MLP per la testa storia
        # applicato a tutti gli h hidden state (batch, h, gru_hidden)
        # nn.Linear/nn.Sequential agiscono sull'ultima dim => ok su tensori 3D
        self.mlp_history = nn.Sequential(
            nn.Linear(gru_hidden, mlp_hidden),
            nn.ReLU(),
            nn.Linear(mlp_hidden, mlp_hidden // 2),
            nn.ReLU(),
        )

        # Stadio 2b: MLP per la testa futuro
        # applicato all'ultimo hidden state h(t) (batch, gru_hidden)
        self.mlp_future = nn.Sequential(
            nn.Linear(gru_hidden, mlp_hidden),
            nn.ReLU(),
            nn.Linear(mlp_hidden, mlp_hidden // 2),
            nn.ReLU(),
        )

        # Testa storia: (batch, h, mlp_hidden//2) -> (batch, h, N_OUTPUTS)
        # predice tail_target_rad per ogni istante passato
        self.head_history = nn.Linear(mlp_hidden // 2, N_OUTPUTS)

        # Testa futuro: (batch, mlp_hidden//2) -> (batch, N_OUTPUTS)
        # predice tail_target_rad al timestep t+1
        self.head_future = nn.Linear(mlp_hidden // 2, N_OUTPUTS)

    def forward(self, seq):
        """
        seq:     (batch, h, input_size)   => storia sensoriale normalizzata
                                             [sensor_diff, current]

        returns:
            pred_history  (batch, h, N_OUTPUTS)   => comandi agli ultimi h istanti passati
            pred_future   (batch, N_OUTPUTS)      => comando al timestep t+1
            h             (batch, gru_hidden)     => contesto temporale
        """
        # all_h: (batch, h, gru_hidden) — tutti gli hidden state
        # h_n:   (1, batch, gru_hidden)     — solo l'ultimo
        all_h, h_n = self.gru(seq)
        h = h_n.squeeze(0)   # (batch, gru_hidden)

        # testa storia: MLP su tutti gli hidden state -> comandi passati
        x_hist = self.mlp_history(all_h)                 # (batch, h, mlp_hidden//2)
        pred_history = self.head_history(x_hist)         # (batch, h, N_OUTPUTS)

        # testa futuro: MLP sull'ultimo hidden state -> comando t+1
        x_fut = self.mlp_future(h)                        # (batch, mlp_hidden//2)
        pred_future = self.head_future(x_fut)            # (batch, N_OUTPUTS)

        return pred_history, pred_future, h