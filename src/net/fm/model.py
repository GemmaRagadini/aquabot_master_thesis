import torch
import torch.nn as nn

# H: lunghezza finestra di storia in ingresso (campioni t-H .. t-1).
# Deve combaciare con dataset.py.
H = 20

# Canali:
#   ingresso RNN : [cmd, sensor_diff, current]  -> 3 canali (storia fino a t-1)
#   ingresso MLP : [h | cmd(t)]
#   uscita       : [sensor_diff, current] a t   -> 2 canali
N_CMD_CHANNELS  = 1
N_SENS_CHANNELS = 2
N_IN_CHANNELS   = N_CMD_CHANNELS + N_SENS_CHANNELS

CELLS = ("gru", "lstm")


class ForwardModel(nn.Module):
    """Forward model a 1 passo (GRU o LSTM + MLP).

      sens(t) = f( cmd[t-H..t-1], sens[t-H..t-1], cmd(t) )

    Stadio 1: rete ricorrente (GRU, oppure LSTM come baseline) sulla storia
              congiunta [cmd, sd, current] fino a t-1 -> h (stato della
              dinamica recente). Lo stato iniziale e' zero in ogni finestra.
    Stadio 2: MLP su [h | cmd(t)] -> sensori a t.

    Nessun contesto statico [amp, freq, center]: sono parametri del generatore
    del comando, gia' ricavabili dalla storia dei comandi. Il modello dipende
    solo dai segnali, quindi resta valido qualunque sia l'origine del comando.

    L'uscita e' direttamente sens(t), in spazio normalizzato.

    Nota: il modulo ricorrente si chiama 'gru' anche quando cell='lstm', cosi'
    i checkpoint GRU gia' salvati si caricano senza modifiche. Per la LSTM,
    gru_hidden e' il numero di unita' della LSTM.

    encode(seq_cmd, seq_sens)         -> (batch, gru_hidden)
    forward(seq_cmd, seq_sens, cmd_t) -> (batch, N_SENS_CHANNELS)
    """

    def __init__(self, gru_hidden=128, mlp_hidden=256, num_layers=1,
                 dropout=0.0, cell="gru"):
        super().__init__()
        if cell not in CELLS:
            raise ValueError(f"cell deve essere una di {CELLS}, non '{cell}'")
        self.cell = cell
        self.gru_hidden = gru_hidden
        self.mlp_hidden = mlp_hidden
        self.num_layers = num_layers

        # Stadio 1: encoder ricorrente della dinamica passata (comandi + sensori).
        rnn = nn.GRU if cell == "gru" else nn.LSTM
        self.gru = rnn(
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
        _, state = self.gru(seq)                        # stato iniziale = zero
        h_n = state[0] if self.cell == "lstm" else state   # LSTM: (h_n, c_n)
        return h_n[-1]                                  # ultimo layer: (B, gru_hidden)

    def forward(self, seq_cmd, seq_sens, cmd_t):
        """seq_cmd (B,H,1), seq_sens (B,H,2), cmd_t (B,1) -> (B,2)"""
        h = self.encode(seq_cmd, seq_sens)
        return self.head(self.mlp(torch.cat([h, cmd_t], dim=-1)))


def build_model(gru_hidden=128, mlp_hidden=256, num_layers=1, dropout=0.0, cell="gru"):
    """Istanzia il forward model (cell = 'gru' o 'lstm')."""
    return ForwardModel(gru_hidden=gru_hidden, mlp_hidden=mlp_hidden,
                        num_layers=num_layers, dropout=dropout, cell=cell)


def load_forward_model(checkpoint, device):
    """Ricostruisce il forward model con gli iperparametri salvati nel checkpoint.
    I checkpoint senza la chiave 'cell' (salvati prima della LSTM) sono GRU."""
    ckpt = torch.load(checkpoint, map_location=device, weights_only=False)
    # I pesi di un modello allenato con uscita residua si caricherebbero senza
    # errori ma darebbero predizioni sbagliate (manca la somma con sens(t-1)).
    if ckpt.get("residual", True):
        raise ValueError(
            f"{checkpoint} e' stato allenato con uscita residua, non piu' "
            f"supportata: riallena con train.py.")
    FM = build_model(gru_hidden=ckpt["gru_hidden"], mlp_hidden=ckpt["mlp_hidden"],
                     num_layers=ckpt.get("num_layers", 1), cell=ckpt.get("cell", "gru"))
    FM.load_state_dict(ckpt["fm_state"])
    FM.to(device).eval()
    print(f"Modello da {checkpoint}: {FM.cell.upper()} {FM.gru_hidden}x{FM.num_layers} | "
          f"MLP {FM.mlp_hidden} | H={ckpt.get('H')}")
    return FM, ckpt