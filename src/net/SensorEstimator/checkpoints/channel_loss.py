"""
Loss per canale sul validation set (split per-trial, lo stesso del training).

Per ogni canale [sensor_diff, sensor_mean, current] stampa:
  - MSE normalizzata (target ~ std 1, quindi MSE~1 = "non batte la media", MSE~0 = perfetto)
  - R2 approssimato = 1 - MSE  (quota di varianza spiegata)
  - MSE della persistenza (baseline: "predici il valore precedente")
Cosi' si vede subito se un canale gonfia la media o se il modello e' mediocre ovunque.

Le dimensioni del modello (gru_hidden, mlp_hidden) sono lette DALLE SHAPE dei pesi
nel checkpoint: niente flag da passare a mano, niente rischio di size mismatch.

Uso:
python3 src/net/SensorEstimator/checkpoints/channel_loss.py [--json_out channel_loss.json]

"""
import argparse
import json
import os
import sys

import numpy as np
import torch
from torch.utils.data import DataLoader

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
REPO_ROOT  = os.path.abspath(os.path.join(SCRIPT_DIR, "..", "..", "..", ".."))
sys.path.insert(0, os.path.join(REPO_ROOT, "src"))

from net.SensorEstimator.model import FishSensorEstimator
from net.SensorEstimator.dataset import FishDataset

CHANNELS_ALL = ["sensor_diff", "sensor_mean", "current"]
# se sensor_mean e' escluso dai target, i canali sono [sensor_diff, current]
CHANNELS_2 = ["sensor_diff", "current"]


def dims_from_state_dict(sd):
    """Ricostruisce gru_hidden e mlp_hidden dalle shape dei pesi salvati,
    cosi' non serve passarli a mano ne' modificare train.py per salvarli.

      gru.weight_hh_l0 : (3*gru_hidden, gru_hidden)  -> gru_hidden = shape[1]
      mlp.0.weight     : (mlp_hidden,   gru_hidden)  -> mlp_hidden = shape[0]
    """
    gru_hidden = sd["gru.weight_hh_l0"].shape[1]
    mlp_hidden = sd["mlp.0.weight"].shape[0]
    return int(gru_hidden), int(mlp_hidden)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--checkpoint", default=os.path.join(SCRIPT_DIR, "best.pt"))
    parser.add_argument("--dataset_dir", default=os.path.join(REPO_ROOT, "src", "net", "dataset"))
    parser.add_argument("--scaler_path", default=os.path.join(REPO_ROOT, "src", "net", "scaler", "scalers.pkl"),
                         help="normalizzatore da riusare: DEVE essere lo stesso del training")
    parser.add_argument("--device", default="cuda" if torch.cuda.is_available() else "cpu")
    parser.add_argument("--json_out", default=None,
                         help="se dato, scrive qui la tabella MSE storia/futuro/persist in JSON "
                              "(formato compatibile con convert_mse_to_real_units.py --mse-json). "
                              "Passa 'auto' per scriverla accanto al checkpoint "
                              "(<checkpoint_dir>/channel_loss.json).")
    args = parser.parse_args()

    device = torch.device(args.device)

    # stessa normalizzazione del training (senza scaler_path i numeri non sarebbero confrontabili)
    dataset = FishDataset(args.dataset_dir, scaler_path=args.scaler_path)

    # STESSO split del training: uso il metodo del dataset (stesso seed, stesso shuffle).
    # Non ricreo la logica altrove per non rischiare uno split diverso.
    _, val_ds = dataset.split_by_trial(val_frac=0.2, seed=42)
    val_loader = DataLoader(val_ds, batch_size=256)

    ckpt = torch.load(args.checkpoint, map_location=device)
    state = ckpt["model_state"]
    input_size = ckpt.get("input_size", dataset.sequences.shape[-1])
    gru_hidden, mlp_hidden = dims_from_state_dict(state)
    print(f"input_size={input_size} | gru_hidden={gru_hidden} | mlp_hidden={mlp_hidden} "
          f"(letti dal checkpoint)")

    model = FishSensorEstimator(input_size=input_size,
                                 gru_hidden=gru_hidden,
                                 mlp_hidden=mlp_hidden,
                                 h=dataset.h).to(device)
    model.load_state_dict(state)
    model.eval()

    # numero di canali predetti: letto dai target del dataset (2 o 3)
    nc = dataset.targets_future.shape[-1]
    CHANNELS = CHANNELS_2 if nc == 2 else CHANNELS_ALL
    print(f"canali predetti: {nc} -> {CHANNELS}")

    # accumulatori errore quadratico per canale, separati per testa
    se_fut = torch.zeros(nc); n_fut = 0
    se_his = torch.zeros(nc); n_his = 0
    # baseline persistenza sulla history: predici t-1 al posto di t (per canale)
    se_persist = torch.zeros(nc); n_persist = 0

    with torch.no_grad():
        for seq, t_hist, t_fut, _ in val_loader:
            seq    = seq.to(device)
            t_hist = t_hist.to(device)
            t_fut  = t_fut.to(device)

            pred_history, pred_future, _ = model(seq)

            se_fut += ((pred_future - t_fut) ** 2).sum(dim=0).cpu()
            n_fut  += t_fut.shape[0]

            se_his += ((pred_history - t_hist) ** 2).reshape(-1, nc).sum(dim=0).cpu()
            n_his  += t_hist.shape[0] * t_hist.shape[1]

            # persistenza: confronto t_hist[:, 1:] con t_hist[:, :-1] (shift di 1 nel tempo)
            if t_hist.shape[1] >= 2:
                diff = (t_hist[:, 1:, :] - t_hist[:, :-1, :]) ** 2
                se_persist += diff.reshape(-1, nc).sum(dim=0).cpu()
                n_persist  += diff.shape[0] * diff.shape[1]

    mse_fut     = (se_fut / n_fut).numpy()
    mse_his     = (se_his / n_his).numpy()
    mse_persist = (se_persist / n_persist).numpy() if n_persist else np.full(nc, np.nan)

    # R2 approssimato: target normalizzati (var ~ 1) -> R2 ~ 1 - MSE
    r2_his = 1.0 - mse_his

    print(f"\n{'canale':<14}{'MSE storia':>12}{'R2~':>8}{'MSE futuro':>12}{'MSE persist':>13}")
    print("-" * 59)
    for c in range(nc):
        print(f"{CHANNELS[c]:<14}{mse_his[c]:>12.4f}{r2_his[c]:>8.3f}"
              f"{mse_fut[c]:>12.4f}{mse_persist[c]:>13.4f}")
    print("-" * 59)
    print(f"{'media':<14}{mse_his.mean():>12.4f}{r2_his.mean():>8.3f}"
          f"{mse_fut.mean():>12.4f}{mse_persist.mean():>13.4f}")

    # --- export JSON, stessa struttura attesa da convert_mse_to_real_units.py (--mse-json) ---
    if args.json_out:
        out_path = args.json_out
        if out_path == "auto":
            out_path = os.path.join(os.path.dirname(os.path.abspath(args.checkpoint)), "channel_loss.json")

        table = {
            CHANNELS[c]: {
                "storia":  float(mse_his[c]),
                "futuro":  float(mse_fut[c]),
                "persist": float(mse_persist[c]) if n_persist else None,
            }
            for c in range(nc)
        }

        os.makedirs(os.path.dirname(out_path) or ".", exist_ok=True)
        with open(out_path, "w") as f:
            json.dump(table, f, indent=2)
        print(f"\nTabella MSE salvata in {out_path}")


if __name__ == "__main__":
    main()