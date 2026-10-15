"""
Controllo dell'AMPIEZZA predetta dal forward model, a un passo e in rollout.

Per ogni trial di validation e per ogni canale calcola:
  rapporto std   = std(predetto) / std(vero)          (1 = ampiezza giusta)
  rapporto range = (p95 - p5)(predetto) / (p95 - p5)(vero)   (piu' robusto ai picchi)
sia per la predizione a un passo (storia vera, teacher forced) sia per il
rollout su tutto il trial (stesso rollout di rollout_test.py).

Come leggerlo:
  - rollout > 1 su (quasi) tutti i trial, a un passo ~1
        -> il rollout AMPLIFICA le oscillazioni: effetto del training solo a
           un passo (le proprie predizioni rientrano in ingresso)
  - rapporti sparsi attorno a 1, sia a un passo sia in rollout
        -> variabilita' tra trial: il modello impara una risposta media e
           non puo' sapere che in quel trial il robot risponde di piu'/meno
  - > 1 gia' a un passo
        -> sovrastima del modello in se', non del rollout

Uso (dalla cartella dei test, accanto a rollout_test.py):
  python3 src/net/fm/tests/rollout_amplitude.py --checkpoint src/net/fm/checkpoints_fm/best_fm_tuned.pt
  python3 ... --csv_out amplitude.csv
"""
import argparse
import os
import sys

import numpy as np
import pandas as pd
import torch

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, SCRIPT_DIR)

# rollout_test.py sistema anche sys.path per net.*; il suo main non viene eseguito
from rollout_test import (FM_CHANNELS, REPO_ROOT, SPLIT_SEED, VAL_FRAC,  # noqa: E402
                          load_model, rollout_trial)
from net.dataset import FishJointDataset  # noqa: E402


@torch.no_grad()
def one_step_trial(dataset, FM, device, trial_idx, batch_size=256):
    """Predizioni a un passo (storia vera) su tutte le finestre del trial: (n,2)."""
    idxs = np.sort(np.nonzero(dataset.window_trial == trial_idx)[0])
    seq_cmd, seq_sens = dataset.seq_cmd[idxs], dataset.seq_sens[idxs]
    cmd_t = dataset.tgt_cmd[idxs][:, 0, :]
    preds = []
    for s in range(0, len(idxs), batch_size):
        sl = slice(s, s + batch_size)
        preds.append(FM(seq_cmd[sl].to(device), seq_sens[sl].to(device),
                        cmd_t[sl].to(device)).cpu())
    return torch.cat(preds, 0).numpy()


def amp_ratios(pred, true):
    """(rapporto std, rapporto range p5-p95) per un canale."""
    std_r = float(np.std(pred) / np.std(true)) if np.std(true) > 0 else float("nan")
    rng = lambda a: float(np.percentile(a, 95) - np.percentile(a, 5))
    rng_r = rng(pred) / rng(true) if rng(true) > 0 else float("nan")
    return std_r, rng_r


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--checkpoint", required=True)
    ap.add_argument("--dataset_dir", default=os.path.join(REPO_ROOT, "src", "net", "dataset"))
    ap.add_argument("--scaler_path", default=os.path.join(REPO_ROOT, "src", "net", "scaler",
                                                          "scalers_joint.pkl"))
    ap.add_argument("--device", default="cuda" if torch.cuda.is_available() else "cpu")
    ap.add_argument("--csv_out", default=None, help="salva la tabella per trial in CSV")
    args = ap.parse_args()

    device = torch.device(args.device)
    FM, ckpt = load_model(args.checkpoint, device)
    kwargs = {"scaler_path": args.scaler_path, "p": 1}
    if ckpt.get("H") is not None:
        kwargs["h"] = int(ckpt["H"])
    dataset = FishJointDataset(args.dataset_dir, **kwargs)
    _, val_ds = dataset.split_by_trial(val_frac=VAL_FRAC, seed=SPLIT_SEED)
    trials = sorted(int(i) for i in np.unique(dataset.window_trial[np.asarray(val_ds.indices)]))

    rows = []
    for k, ti in enumerate(trials, start=1):
        print(f"\r  trial {k}/{len(trials)}: {dataset.trial_names[ti]}        ", end="", flush=True)
        roll, true = rollout_trial(dataset, FM, device, ti)
        one = one_step_trial(dataset, FM, device, ti)
        row = {"trial": dataset.trial_names[ti]}
        for ci, ch in enumerate(FM_CHANNELS):
            s1, r1 = amp_ratios(one[:, ci], true[:, ci])
            sr, rr = amp_ratios(roll[:, ci], true[:, ci])
            row.update({f"{ch}_std_1": s1, f"{ch}_std_R": sr,
                        f"{ch}_range_1": r1, f"{ch}_range_R": rr})
        rows.append(row)
    print()

    df = pd.DataFrame(rows)
    pd.set_option("display.width", 220)
    pd.set_option("display.max_columns", None)
    print("\nRapporti di ampiezza predetto/vero per trial (1 = ampiezza giusta; "
          "_1 = un passo, _R = rollout):")
    print(df.round(3).to_string(index=False))

    print("\nRiepilogo sui trial di validation:")
    summ = []
    for col in [c for c in df.columns if c != "trial"]:
        v = df[col].to_numpy()
        summ.append({"rapporto": col, "media": v.mean(), "min": v.min(), "max": v.max(),
                     "trial > 1": f"{int((v > 1).sum())}/{len(v)}"})
    print(pd.DataFrame(summ).round(3).to_string(index=False))

    if args.csv_out:
        df.to_csv(args.csv_out, index=False)
        print(f"\nSalvato {args.csv_out}")


if __name__ == "__main__":
    main()