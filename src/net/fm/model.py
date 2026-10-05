import torch
import torch.nn as nn

# H: lunghezza finestra di storia in ingresso (campioni t-H .. t-1).
# Deve combaciare con dataset.py.
H = 20

# Canali:
#   ingresso GRU : [cmd, sensor_diff, current]  -> 3 canali (storia fino a t-1)
#   ingresso MLP : [h_gru | cmd(t)]
#   uscita       : [sensor_diff, current] a t   -> 2 canali
N_CMD_CHANNELS  = 1
N_SENS_CHANNELS = 2
N_IN_CHANNELS   = N_CMD_CHANNELS + N_SENS_CHANNELS


class ForwardModel(nn.Module):
    """Forward model a 1 passo (GRU + MLP).

      sens(t) = f( cmd[t-H..t-1], sens[t-H..t-1], cmd(t) )

    Stadio 1: GRU sulla storia congiunta [cmd, sd, current] fino a t-1 -> h
              (stato della dinamica recente).
    Stadio 2: MLP su [h | cmd(t)] -> sensori a t.

    Nessun contesto statico [amp, freq, center]: sono parametri del generatore
    del comando, gia' ricavabili dalla storia dei comandi. Il modello dipende
    solo dai segnali, quindi resta valido qualunque sia l'origine del comando.

    L'uscita e' direttamente sens(t), in spazio normalizzato.

    encode(seq_cmd, seq_sens)       -> (batch, gru_hidden)
    forward(seq_cmd, seq_sens, cmd_t) -> (batch, N_SENS_CHANNELS)
    """

    def __init__(self, gru_hidden=128, mlp_hidden=256, num_layers=1,
                 dropout=0.0):
        super().__init__()
        self.gru_hidden = gru_hidden
        self.mlp_hidden = mlp_hidden
        self.num_layers = num_layers

        # Stadio 1: GRU — encoder della dinamica passata (comandi + sensori).
        self.gru = nn.GRU(
            input_size=N_IN_CHANNELS,
            hidden_size=gru_hidden,
            num_layers=num_layers,
            batch_first=True,
            dropout=dropout if num_layers > 1 else 0.0,
        )

        # Stadio 2: MLP su [hidden | comando a t].
        mlp_in = gru_hidden + N_CMD_CHANNELS
        self.mlp = nn.Sequential(
            nn.Linear(mlp_in, mlp_hidden),
            nn.ReLU(),
            nn.Dropout(dropout),
            nn.Linear(mlp_hidden, mlp_hidden // 2),
            nn.ReLU(),
            nn.Dropout(dropout),
        )

        # Testa: sensori a t.
        self.head = nn.Linear(mlp_hidden // 2, N_SENS_CHANNELS)

    def encode(self, seq_cmd, seq_sens):
        seq = torch.cat([seq_cmd, seq_sens], dim=-1)   # (B, H, 3)
        _, h_n = self.gru(seq)                          # (num_layers, B, gru_hidden)
        return h_n[-1]                                  # ultimo layer: (B, gru_hidden)

    def forward(self, seq_cmd, seq_sens, cmd_t):
        """seq_cmd (B,H,1), seq_sens (B,H,2), cmd_t (B,1) -> (B,2)"""
        h = self.encode(seq_cmd, seq_sens)
        return self.head(self.mlp(torch.cat([h, cmd_t], dim=-1)))


def build_model(gru_hidden=128, mlp_hidden=256, num_layers=1, dropout=0.0):
    """Istanzia il forward model."""
    return ForwardModel(gru_hidden=gru_hidden, mlp_hidden=mlp_hidden,
                        num_layers=num_layers, dropout=dropout)