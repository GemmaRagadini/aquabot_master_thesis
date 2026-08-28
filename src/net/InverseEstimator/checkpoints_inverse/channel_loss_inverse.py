"""
Loss per canale + conversione in unita' reali 

-------
1. Ricarica il checkpoint dell'inverse (dimensioni lette dalle shape dei pesi,
   nessun flag da passare a mano).
2. Calcola per il canale cmd:
     - MSE normalizzata (target ~ std 1, quindi MSE~1 = "non batte la media",
       MSE~0 = perfetto)
     - R2 approssimato = 1 - MSE
     - MSE della persistenza (baseline: "predici il comando precedente")
   separatamente per la testa storia e la testa futuro.
3. Converte MSE/RMSE normalizzate in unita' reali (rad) usando lo scaler 'cmd':
       x_norm = (x - mean) / std   =>   RMSE_reale = RMSE_norm * std
                                        MSE_reale  = MSE_norm  * std**2

Uso
---
  python3 src/net/InverseEstimator/checkpoints_inverse/channel_loss_inverse.py [--json_out auto]


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

from net.InverseEstimator.model_inverse   import FishInverseEstimator
from net.InverseEstimator.dataset_inverse import FishInverseDataset

# la rete inversa ha un solo canale di output: il comando servo.
# tenuto come lista per restare simmetrico agli script della diretta (che iterano
# sui canali) e per non dover cambiare la struttura se un domani si aggiungono uscite.
CHANNELS = ["cmd"]

# nome canale -> chiave dello scaler in scalers_inverse.pkl
CHANNEL_TO_SCALER_KEY = {"cmd": "cmd"}
# nome canale -> unita' fisica reale
CHANNEL_UNIT = {"cmd": "rad"}


def dims_from_state_dict(sd):
    """Ricostruisce gru_hidden e mlp_hidden dalle shape dei pesi salvati,
    identico agli script della diretta (stessa struttura di nomi nel modello).

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
    parser.add_argument("--scaler_path", default=os.path.join(REPO_ROOT, "src", "net", "scaler", "scalers_inverse.pkl"),
                         help="normalizzatore dell'inverse da riusare: DEVE essere lo stesso del training")
    parser.add_argument("--device", default="cuda" if torch.cuda.is_available() else "cpu")
    parser.add_argument("--json_out", default=None,
                         help="se dato, scrive qui la tabella MSE storia/futuro/persist in JSON "
                              "(stessa struttura degli script della diretta). "
                              "Passa 'auto' per scriverla accanto al checkpoint "
                              "(<checkpoint_dir>/channel_loss_inverse.json).")
    args = parser.parse_args()

    device = torch.device(args.device)

    # stessa normalizzazione del training (senza scaler_path i numeri non sarebbero confrontabili)
    dataset = FishInverseDataset(args.dataset_dir, scaler_path=args.scaler_path)

    # STESSO split del training: uso dataset.split_by_trial (stesso seed, stesso
    # metodo). Non ricreo la logica altrove per non rischiare uno split diverso.
    # Questa chiamata costruisce anche le finestre e fitta lo scaler sul train.
    _, val_ds = dataset.split_by_trial(val_frac=0.2, seed=42)
    val_loader = DataLoader(val_ds, batch_size=256)

    ckpt = torch.load(args.checkpoint, map_location=device)
    state = ckpt["model_state"]
    input_size = ckpt.get("input_size", dataset.sequences.shape[-1])
    gru_hidden, mlp_hidden = dims_from_state_dict(state)
    print(f"input_size={input_size} | gru_hidden={gru_hidden} | mlp_hidden={mlp_hidden} "
          f"(letti dal checkpoint)")

    model = FishInverseEstimator(input_size=input_size,
                                  gru_hidden=gru_hidden,
                                  mlp_hidden=mlp_hidden,
                                  h=dataset.h).to(device)
    model.load_state_dict(state)
    model.eval()

    # numero di canali predetti: letto dai target del dataset (atteso 1 per l'inverse)
    nc = dataset.targets_future.shape[-1]
    if nc != len(CHANNELS):
        print(f"[avviso] il dataset predice {nc} canali ma CHANNELS ne elenca "
              f"{len(CHANNELS)} ({CHANNELS}). Uso i primi {nc}.", file=sys.stderr)
    channels = CHANNELS[:nc]
    print(f"canali predetti: {nc} -> {channels}")

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

    # --- tabella in scala normalizzata ---
    print(f"\n=== MSE normalizzata (validation) ===")
    print(f"{'canale':<10}{'MSE storia':>12}{'R2~':>8}{'MSE futuro':>12}{'MSE persist':>13}")
    print("-" * 55)
    for c in range(nc):
        print(f"{channels[c]:<10}{mse_his[c]:>12.4f}{r2_his[c]:>8.3f}"
              f"{mse_fut[c]:>12.4f}{mse_persist[c]:>13.4f}")

    print("\nLettura:")
    print("  MSE storia ~ 1.0  -> il modello non batte la media (canale non imparato)")
    print("  MSE storia ~ 0.0  -> predizione quasi perfetta")
    print("  MSE persist: se il modello NON e' sotto la persistenza, non aggiunge valore.")

    # --- conversione in unita' reali (rad) usando lo scaler 'cmd' ---
    scalers = dataset.scalers
    print(f"\n=== Conversione in unita' reali ===")
    print(f"{'canale':<10}{'metrica':<10}{'MSE norm':>10}{'RMSE norm':>12}{'RMSE reale':>14}   unita'")
    print("-" * 70)

    results = {}
    for c in range(nc):
        ch = channels[c]
        key = CHANNEL_TO_SCALER_KEY.get(ch)
        if key is None or key not in scalers:
            print(f"[skip] canale '{ch}': nessuno scaler corrispondente "
                  f"(chiavi disponibili: {list(scalers.keys())})", file=sys.stderr)
            continue
        std = float(scalers[key].scale_[0])
        unit = CHANNEL_UNIT.get(ch, "")

        metriche = {
            "storia":  float(mse_his[c]),
            "futuro":  float(mse_fut[c]),
            "persist": float(mse_persist[c]) if n_persist else None,
        }

        results[ch] = {}
        for metrica, mse_n in metriche.items():
            if mse_n is None:
                continue
            rmse_n    = float(np.sqrt(mse_n))
            rmse_real = rmse_n * std
            mse_real  = mse_n * std ** 2
            results[ch][metrica] = {"mse_norm": mse_n, "rmse_norm": rmse_n,
                                     "rmse_real": rmse_real, "mse_real": mse_real, "std": std}
            print(f"{ch:<10}{metrica:<10}{mse_n:>10.4f}{rmse_n:>12.4f}{rmse_real:>14.3f}   {unit}")
        print()

    # --- export JSON opzionale (stessa struttura degli script della diretta) ---
    if args.json_out:
        out_path = args.json_out
        if out_path == "auto":
            out_path = os.path.join(os.path.dirname(os.path.abspath(args.checkpoint)),
                                    "channel_loss_inverse.json")

        table = {
            channels[c]: {
                "storia":  float(mse_his[c]),
                "futuro":  float(mse_fut[c]),
                "persist": float(mse_persist[c]) if n_persist else None,
            }
            for c in range(nc)
        }

        os.makedirs(os.path.dirname(out_path) or ".", exist_ok=True)
        with open(out_path, "w") as f:
            json.dump(table, f, indent=2)
        print(f"Tabella MSE (norm) salvata in {out_path}")


if __name__ == "__main__":
    main()