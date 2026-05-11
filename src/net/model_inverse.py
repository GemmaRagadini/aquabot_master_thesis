import torch
import torch.nn as nn

# H_OUT: quanti istanti passati predice la testa storia
# a 20 Hz, 20 timestep = 1 secondo = un ciclo completo a 1 Hz
H_OUT = 20


class FishInverseEstimator(nn.Module):
    def __init__(self, input_size=3, gru_hidden=64, mlp_hidden=128, h_out=H_OUT):
        """
        Stimatore inverso: data una finestra temporale di letture sensoriali,
        predice il comando motore necessario per produrle.

        input_size:  numero di canali in ingresso (3 => sensor_diff, sensor_mean, v_flow)
        gru_hidden:  dimensione hidden state GRU
        mlp_hidden:  dimensione hidden layer MLP
        h_out:       quanti istanti passati predice la testa storia
        """
        super().__init__()
        self.h_out = h_out

        # Stadio 1: encoder temporale
        # input:  (batch, window, 3)  =>  storia dei sensori
        # output: tutti gli hidden state (batch, window, gru_hidden)
        #         + ultimo hidden state  (batch, gru_hidden)
        self.gru = nn.GRU(
            input_size=input_size,
            hidden_size=gru_hidden,
            num_layers=1,
            batch_first=True,
        )

        # Stadio 2: MLP — usato solo per la testa futuro
        # input: h(t)  =>  dim gru_hidden
        self.mlp = nn.Sequential(
            nn.Linear(gru_hidden, mlp_hidden),
            nn.ReLU(),
            nn.Linear(mlp_hidden, mlp_hidden // 2),
            nn.ReLU(),
        )

        # Testa storia: applicata su ogni hidden state degli ultimi h_out timestep
        # (batch, h_out, gru_hidden) -> (batch, h_out, 1) -> squeeze -> (batch, h_out)
        # predice tail_target_rad per ogni istante passato
        self.head_history = nn.Linear(gru_hidden, 1)

        # Testa futuro: predice tail_target_rad al timestep t+1
        # usata a runtime per chiudere il loop di controllo
        # (batch, mlp_hidden//2) -> (batch,)
        self.head_future = nn.Linear(mlp_hidden // 2, 1)

    def forward(self, seq):
        """
        seq:     (batch, window, 3)   =>  storia sensori normalizzati
                                          [sensor_diff, sensor_mean, v_flow]
        returns:
            cmd_history  (batch, h_out)   =>  comandi passati agli ultimi h_out istanti
            cmd_future   (batch,)         =>  comando al timestep t+1  [usato a runtime]
            h            (batch, gru_hidden) =>  contesto temporale
        """
        # all_h: (batch, window, gru_hidden) — tutti gli hidden state
        # h_n:   (1, batch, gru_hidden)      — solo l'ultimo
        all_h, h_n = self.gru(seq)
        h = h_n.squeeze(0)   # (batch, gru_hidden)

        # testa storia: ultimi h_out hidden state -> comandi passati
        cmd_history = self.head_history(all_h[:, -self.h_out:, :]).squeeze(-1)  # (batch, h_out)

        # testa futuro: ultimo hidden state -> MLP -> comando t+1
        x = self.mlp(h)
        cmd_future = self.head_future(x).squeeze(1)   # (batch,)

        return cmd_history, cmd_future, h