import torch
import torch.nn as nn

# H: lunghezza finestra di storia in ingresso. P: orizzonte di predizione.
# Devono combaciare con dataset_joint.py.
H = 20
P = 1

CTX_DIM = 4

# Canali dell'ingresso condiviso [C_T, S_T]:
#   cmd (1) + sensori (2) = 3 canali per timestep, per ENTRAMBE le reti.
N_CMD_CHANNELS  = 1
N_SENS_CHANNELS = 2
N_INPUT_CHANNELS = N_CMD_CHANNELS + N_SENS_CHANNELS   # = 3


class GRUMLPEstimator(nn.Module):
    """Stimatore GRU+MLP a testa singola che predice P passi in un colpo solo.

    Usato per entrambe le reti (istanziato due volte):
      IM (inversa): out_channels = N_CMD_CHANNELS  (1)  -> predice comandi futuri
      FM (diretta): out_channels = N_SENS_CHANNELS (2)  -> predice sensori futuri

    Ingresso IDENTICO per le due reti:
      seq: (batch, H, N_INPUT_CHANNELS)  => [C_T, S_T] concatenati sui canali
      ctx: (batch, ctx_dim)              => contesto statico (solo MLP, non GRU)

    Uscita:
      pred: (batch, P, out_channels)     => P passi futuri
      h:    (batch, gru_hidden)          => hidden finale (utile in Fase B)
    """

    def __init__(self, out_channels, in_channels=N_INPUT_CHANNELS,
                 gru_hidden=256, mlp_hidden=256, p=P, ctx_dim=CTX_DIM):
        super().__init__()
        self.p = p
        self.out_channels = out_channels
        self.ctx_dim = ctx_dim

        # Stadio 1: GRU — encoder temporale della sola dinamica osservata.
        # Il contesto NON entra qui (resta un encoder della storia grezza).
        self.gru = nn.GRU(
            input_size=in_channels,
            hidden_size=gru_hidden,
            num_layers=1,
            batch_first=True,
        )

        # Stadio 2: MLP sull'ultimo hidden state + contesto.
        self.mlp = nn.Sequential(
            nn.Linear(gru_hidden + ctx_dim, mlp_hidden),
            nn.ReLU(),
            nn.Linear(mlp_hidden, mlp_hidden // 2),
            nn.ReLU(),
        )

        # Testa unica: proietta a P * out_channels, poi reshape a (batch, P, out_channels).
        self.head = nn.Linear(mlp_hidden // 2, p * out_channels)

    def forward(self, seq, ctx):
        _, h_n = self.gru(seq)          # h_n: (1, batch, gru_hidden)
        h = h_n.squeeze(0)              # (batch, gru_hidden)

        x = self.mlp(torch.cat([h, ctx], dim=-1))   # (batch, mlp_hidden//2)
        flat = self.head(x)                          # (batch, P*out_channels)
        pred = flat.view(-1, self.p, self.out_channels)   # (batch, P, out_channels)
        return pred, h


def build_models(gru_hidden_im=128, mlp_hidden_im=64,
                 gru_hidden_fm=256, mlp_hidden_fm=256,
                 p=P, ctx_dim=CTX_DIM):
    """Istanzia le due reti con lo stesso ingresso condiviso.
    IM predice comandi (1 canale), FM predice sensori (2 canali)."""
    IM = GRUMLPEstimator(out_channels=N_CMD_CHANNELS,
                         gru_hidden=gru_hidden_im, mlp_hidden=mlp_hidden_im,
                         p=p, ctx_dim=ctx_dim)
    FM = GRUMLPEstimator(out_channels=N_SENS_CHANNELS,
                         gru_hidden=gru_hidden_fm, mlp_hidden=mlp_hidden_fm,
                         p=p, ctx_dim=ctx_dim)
    return IM, FM