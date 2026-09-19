import torch
import torch.nn as nn

# H: lunghezza finestra di storia in ingresso. P: orizzonte di predizione.
# Devono combaciare con dataset.py.
H = 20
P = 10

# Contesto statico condiviso [amp, freq, center] iniettato in ENTRAMBI gli MLP.
CTX_DIM = 3

# Canali temporali (GRU):
#   IM (inversa): GRU sui comandi  -> 1 canale
#   FM (diretta): GRU sui sensori  -> 2 canali
N_CMD_CHANNELS  = 1
N_SENS_CHANNELS = 2


class GRUMLPEstimator(nn.Module):
    """Stimatore GRU+MLP con HIDDEN STATE INCROCIATO tra le due reti.

    Ogni rete codifica il proprio segnale con la sua GRU; il suo MLP riceve
    l'hidden PROPRIO + l'hidden dell'ALTRA rete + il contesto statico ctx
    [amp, freq, center]:

      IM: GRU_IM(comandi) -> h_im ; MLP_IM([h_im, h_fm, ctx]) -> comandi futuri
      FM: GRU_FM(sensori) -> h_fm ; MLP_FM([h_fm, h_im, ctx]) -> sensori futuri

    L'hidden incrociato viene calcolato dall'ALTRA rete e passato dall'esterno
    (train.py: prima si fa encode() su entrambe, poi decode() incrociando).
    Cosi' i gradienti di loss_im fluiscono anche in GRU_FM e viceversa: le due
    reti restano accoppiate in training, coerentemente con lo schema.

    encode(seq)               -> (batch, gru_hidden)      hidden proprio
    decode(own_h, cross_h,ctx)-> (batch, P, out_channels) predizione
    forward(seq, cross_h, ctx)-> (pred, own_h)            encode+decode in uno
    """

    def __init__(self, out_channels, in_channels, cross_hidden,
                 gru_hidden=256, mlp_hidden=256, p=P, ctx_static=CTX_DIM,
                 dropout=0.0):
        super().__init__()
        self.p = p
        self.out_channels = out_channels
        self.gru_hidden   = gru_hidden
        self.cross_hidden = cross_hidden
        self.ctx_static   = ctx_static

        # Stadio 1: GRU — encoder temporale della sola dinamica "propria".
        self.gru = nn.GRU(
            input_size=in_channels,
            hidden_size=gru_hidden,
            num_layers=1,
            batch_first=True,
        )

        # Stadio 2: MLP su [hidden proprio | hidden incrociato | contesto statico].
        mlp_in = gru_hidden + cross_hidden + ctx_static
        self.mlp = nn.Sequential(
            nn.Linear(mlp_in, mlp_hidden),
            nn.ReLU(),
            nn.Dropout(dropout),
            nn.Linear(mlp_hidden, mlp_hidden // 2),
            nn.ReLU(),
            nn.Dropout(dropout),
        )

        # Testa unica: P * out_channels, poi reshape a (batch, P, out_channels).
        self.head = nn.Linear(mlp_hidden // 2, p * out_channels)

    def encode(self, seq):
        _, h_n = self.gru(seq)          # h_n: (1, batch, gru_hidden)
        return h_n.squeeze(0)           # (batch, gru_hidden)

    def decode(self, own_h, cross_h, ctx):
        x = self.mlp(torch.cat([own_h, cross_h, ctx], dim=-1))
        flat = self.head(x)
        return flat.view(-1, self.p, self.out_channels)

    def forward(self, seq, cross_h, ctx):
        own_h = self.encode(seq)
        pred = self.decode(own_h, cross_h, ctx)
        return pred, own_h


def build_models(gru_hidden_im=128, mlp_hidden_im=64,
                 gru_hidden_fm=256, mlp_hidden_fm=256,
                 dropout_im=0.0, dropout_fm=0.0,
                 p=P, ctx_static=CTX_DIM):
    """Istanzia le due reti con hidden state incrociato.

    L'MLP di IM riceve l'hidden di FM (dim = gru_hidden_fm) e viceversa: il
    cross_hidden di ciascuna e' il gru_hidden dell'ALTRA.
    """
    IM = GRUMLPEstimator(out_channels=N_CMD_CHANNELS,  in_channels=N_CMD_CHANNELS,
                         cross_hidden=gru_hidden_fm,
                         gru_hidden=gru_hidden_im, mlp_hidden=mlp_hidden_im,
                         dropout=dropout_im, p=p, ctx_static=ctx_static)

    FM = GRUMLPEstimator(out_channels=N_SENS_CHANNELS, in_channels=N_SENS_CHANNELS,
                         cross_hidden=gru_hidden_im,
                         gru_hidden=gru_hidden_fm, mlp_hidden=mlp_hidden_fm,
                         dropout=dropout_fm, p=p, ctx_static=ctx_static)
    return IM, FM