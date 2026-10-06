"""
Tabella metriche PER TRIAL del FORWARD MODEL a 1 passo.

Il modello predice i sensori [sensor_diff, current] al tempo t a partire da
storia vera di comandi e sensori (t-H .. t-1) + comando vero a t (teacher
forced, non autoregressivo). Per ogni trial si confronta sens(t) predetto col
valore vero, su tutte le finestre del trial.

Cosa calcolare si sceglie con i FLAG; i risultati si accumulano in un CSV
MASTER (merge per chiave trial+split). Senza flag calcola RMSE + nRMSE + R2 e
la baseline di persistenza, sui soli trial di validation.

Flag principali:
  --metrics  rmse mae nrmse r2      quali metriche (default: rmse nrmse r2)
  --baseline persist | none         baseline per skill score (default: persist)
  --channels sensor_diff current    (default: tutti)
  --agg      per_trial | pooled | both  righe riassuntive (default: both)
  --summary_only                    scrivi solo MEDIA/pooled, non i singoli trial
  --all_trials                      includi anche i trial di train
  --overwrite                       ignora il master esistente e riscrivilo

Metriche:
  RMSE, MAE in unita' reali (inverse_transform mono-canale).
  nRMSE = RMSE / std(vero)  (adimensionale; ~0 ottimo, ~1 inutile).
  R2    = 1 - SS_res/SS_tot (1 ottimo, <=0 peggio della media).
  Baseline PERSISTENZA: predizione = sens(t-1). skill = 1 - modello/persist
  (1 = perfetto, 0 = come la persistenza, <0 = peggio).

Righe riassuntive:
  MEDIA per-trial : media delle metriche dei singoli trial (ogni trial pesa uguale)
  POOLED          : metriche su tutte le finestre insieme (trial lunghi pesano di piu')

Uso:
  python3 src/net/fm/metrics.py --checkpoint src/net/fm/checkpoints_fm/best_fm_tuned.pt
  python3 src/net/fm/metrics.py --checkpoint ... --metrics rmse mae nrmse r2 --csv_out metrics.csv
  python3 src/net/fm/metrics.py --checkpoint ... --summary_only        # solo le righe per la slide
"""
import argparse
import os
import sys

import numpy as np
import pandas as pd
import torch

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
REPO_ROOT  = os.path.abspath(os.path.join(SCRIPT_DIR, "..", "..", "..", ".."))
sys.path.insert(0, os.path.join(REPO_ROOT, "src"))

# flat import (come train_fm). In repo: from net.fm.model_fm import ...
from net.fm.model import build_model
from net.dataset  import FishJointDataset

FM_CHANNELS = ["sensor_diff", "current"]
CH_TO_KEY = {"sensor_diff": "sd", "current": "vf"}
CH_UNIT   = {"sensor_diff": "unita' sensore", "current": "mA"}

VAL_FRAC   = 0.2
SPLIT_SEED = 42
ROUND_DEC  = 4

METRIC_LABEL  = {"rmse": "RMSE", "mae": "MAE", "nrmse": "nRMSE", "r2": "R2"}
SKILL_METRICS = ("rmse", "mae")   # skill definito solo su rmse/mae


# ------------------------------- metriche pure -------------------------------
def rmse(pred, true):
    return float(np.sqrt(np.mean((pred - true) ** 2)))

def mae(pred, true):
    return float(np.mean(np.abs(pred - true)))

def nrmse(pred, true):
    s = float(np.std(true))
    return rmse(pred, true) / s if s > 0 else float("nan")

def r2(pred, true):
    ss_res = float(np.sum((true - pred) ** 2))
    ss_tot = float(np.sum((true - np.mean(true)) ** 2))
    return 1.0 - ss_res / ss_tot if ss_tot > 0 else float("nan")

METRIC_FUNCS = {"rmse": rmse, "mae": mae, "nrmse": nrmse, "r2": r2}


# ------------------------------- checkpoint/model -------------------------------
def load_model(checkpoint, device):
    """Ricostruisce il forward model con gli iperparametri salvati nel checkpoint."""
    ckpt = torch.load(checkpoint, map_location=device, weights_only=False)
    # I pesi di un modello allenato con uscita residua si caricherebbero senza
    # errori ma darebbero predizioni sbagliate (manca la somma con sens(t-1)).
    if ckpt.get("residual", True):
        raise ValueError(
            f"{checkpoint} e' stato allenato con uscita residua, non piu' "
            f"supportata: riallena con train_fm.py.")
    FM = build_model(gru_hidden=ckpt["gru_hidden"], mlp_hidden=ckpt["mlp_hidden"],
                     num_layers=ckpt.get("num_layers", 1))
    FM.load_state_dict(ckpt["fm_state"])
    FM.to(device).eval()
    print(f"Modello dal checkpoint: GRU {FM.gru_hidden}x{FM.num_layers} | "
          f"MLP {FM.mlp_hidden} | H={ckpt.get('H')}")
    return FM, ckpt


def run_trial(dataset, FM, device, trial_idx, batch_size=256):
    """Forward su tutte le finestre del trial. Ritorna, in scala normalizzata,
    predizione, target sens(t) e persistenza sens(t-1): array (n, 2)."""
    idxs = np.sort(np.nonzero(dataset.window_trial == trial_idx)[0])
    if len(idxs) == 0:
        return None

    seq_cmd  = dataset.seq_cmd[idxs]
    seq_sens = dataset.seq_sens[idxs]
    cmd_t    = dataset.tgt_cmd[idxs][:, 0, :]     # (n, 1) comando a t
    tgt      = dataset.tgt_sens[idxs][:, 0, :]    # (n, 2) sensori a t

    preds = []
    with torch.no_grad():
        for s in range(0, len(idxs), batch_size):
            sl = slice(s, s + batch_size)
            preds.append(FM(seq_cmd[sl].to(device), seq_sens[sl].to(device),
                            cmd_t[sl].to(device)).cpu())

    return {"pred":    torch.cat(preds, 0).numpy(),
            "true":    tgt.cpu().numpy(),
            "persist": seq_sens[:, -1, :].cpu().numpy()}


def channel_real(out, ch, scaler):
    """(pred_real, true_real, persist_real) di un canale, in unita' reali."""
    ci = FM_CHANNELS.index(ch)

    def inv(a):
        return scaler.inverse_transform(np.asarray(a).reshape(-1, 1)).ravel()

    return inv(out["pred"][:, ci]), inv(out["true"][:, ci]), inv(out["persist"][:, ci])


# ------------------------------- colonne per canale -------------------------------
def channel_columns(ch, pred, true, persist, metrics, baseline):
    """Colonne di un canale: metriche del modello + (rmse/mae) persistenza e skill."""
    cols = {}
    for mt in metrics:
        cols[f"{ch}_{METRIC_LABEL[mt]}"] = METRIC_FUNCS[mt](pred, true)
    if baseline == "persist":
        for mt in [m for m in metrics if m in SKILL_METRICS]:
            mm = METRIC_FUNCS[mt](pred, true)
            pp = METRIC_FUNCS[mt](persist, true)
            cols[f"{ch}_{METRIC_LABEL[mt]}_persist"] = pp
            cols[f"{ch}_skill_{mt}"] = 1.0 - mm / pp if pp > 0 else float("nan")
    return cols


# ------------------------------- merge nel master -------------------------------
def row_sort_key(trial, split):
    if str(trial).startswith("== MEDIA"):
        return (3, str(trial), str(split))
    if str(trial).startswith("== COMPLESSIVO"):
        return (4, str(trial), str(split))
    return ({"val": 0, "train": 1}.get(split, 2), str(trial), str(split))


def merge_into_master(df_new, master_path, overwrite=False):
    """Unisce df_new nel CSV master per chiave (trial, split): le colonne calcolate
    in questo run sovrascrivono/aggiungono quelle vecchie, le altre restano;
    righe nuove vengono aggiunte. Senza master_path ritorna df_new cosi' com'e'."""
    key = ["trial", "split"]
    if overwrite or master_path is None or not os.path.exists(master_path):
        combined = df_new.copy()
    else:
        df_old = pd.read_csv(master_path)
        combined = df_new.set_index(key).combine_first(df_old.set_index(key)).reset_index()

    lead = ["trial", "split", "n_finestre"]
    new_cols = [c for c in df_new.columns if c not in lead]
    rest = [c for c in combined.columns if c not in lead + new_cols]
    combined = combined[[c for c in lead if c in combined.columns]
                        + [c for c in new_cols if c in combined.columns] + rest]

    combined["_sk"] = [row_sort_key(t, s) for t, s in
                       zip(combined["trial"], combined["split"])]
    return combined.sort_values("_sk", kind="stable").drop(columns="_sk") \
                   .reset_index(drop=True)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--checkpoint", default=os.path.join(SCRIPT_DIR, "checkpoints_fm", "best.pt"))
    ap.add_argument("--dataset_dir", default=os.path.join(REPO_ROOT, "src", "net", "dataset"))
    ap.add_argument("--scaler_path", default=os.path.join(REPO_ROOT, "src", "net", "scaler", "scalers_joint.pkl"),
                    help="normalizzatore condiviso: DEVE essere lo stesso del training")
    ap.add_argument("--device", default="cuda" if torch.cuda.is_available() else "cpu")
    ap.add_argument("--all_trials", action="store_true",
                    help="includi anche i trial di train (default: solo validation)")
    ap.add_argument("--csv_out", default=None,
                    help="CSV master: viene aggiornato per merge a ogni run")
    ap.add_argument("--overwrite", action="store_true",
                    help="ignora il master esistente e riscrivilo da zero")

    # --- selezione di COSA calcolare ---
    ap.add_argument("--metrics", nargs="+", default=["rmse", "nrmse", "r2"],
                    choices=["rmse", "mae", "nrmse", "r2"],
                    help="quali metriche (default: rmse nrmse r2).")
    ap.add_argument("--baseline", default="persist", choices=["persist", "none"],
                    help="baseline per lo skill score (default: persist).")
    ap.add_argument("--channels", nargs="+", default=FM_CHANNELS, choices=FM_CHANNELS,
                    help="quali canali (default: tutti).")
    ap.add_argument("--agg", default="both", choices=["per_trial", "pooled", "both"],
                    help="righe riassuntive da aggiungere (default: both).")
    ap.add_argument("--summary_only", action="store_true",
                    help="scrivi solo le righe MEDIA/pooled, non i singoli trial.")
    args = ap.parse_args()

    device = torch.device(args.device)
    FM, ckpt = load_model(args.checkpoint, device)

    ds_kwargs = {"scaler_path": args.scaler_path, "p": 1}
    if ckpt.get("H") is not None:
        ds_kwargs["h"] = int(ckpt["H"])
    dataset = FishJointDataset(args.dataset_dir, **ds_kwargs)

    _, val_ds = dataset.split_by_trial(val_frac=VAL_FRAC, seed=SPLIT_SEED)
    val_trials = set(int(i) for i in np.unique(
        dataset.window_trial[np.asarray(val_ds.indices)]))

    channels = list(dict.fromkeys(args.channels))
    metrics  = list(dict.fromkeys(args.metrics))
    print(f"Metriche: {metrics} | baseline: {args.baseline} | canali: {channels} | "
          f"agg: {args.agg}")

    trials = list(range(len(dataset.trial_names))) if args.all_trials else sorted(val_trials)
    print(f"trial considerati: {len(trials)} "
          f"({'tutti' if args.all_trials else 'solo validation'})")

    pool = {ch: {"pred": [], "true": [], "persist": []} for ch in channels}
    rows, per_trial_metrics = [], []
    for ti in trials:
        out = run_trial(dataset, FM, device, ti)
        if out is None:
            continue
        split = "val" if ti in val_trials else "train"
        row = {"trial": dataset.trial_names[ti], "split": split,
               "n_finestre": len(out["true"])}
        metric_only = {}
        for ch in channels:
            pr, tr, ps = channel_real(out, ch, dataset.scalers[CH_TO_KEY[ch]])
            cols = channel_columns(ch, pr, tr, ps, metrics, args.baseline)
            row.update(cols)
            metric_only.update(cols)
            pool[ch]["pred"].append(pr); pool[ch]["true"].append(tr)
            pool[ch]["persist"].append(ps)
        if not args.summary_only:
            rows.append(row)
        per_trial_metrics.append(metric_only)

    if not per_trial_metrics:
        print("Nessun trial elaborato.", file=sys.stderr)
        sys.exit(1)

    tag = "val" if not args.all_trials else "all"

    # --- riga MEDIA per-trial (ogni trial pesa uguale) ---
    if args.agg in ("per_trial", "both"):
        mean_row = {"trial": "== MEDIA per-trial ==", "split": f"MEDIA_{tag}",
                    "n_finestre": len(per_trial_metrics)}
        for k in per_trial_metrics[0]:
            mean_row[k] = float(np.nanmean([r[k] for r in per_trial_metrics]))
        rows.append(mean_row)

    # --- riga POOLED (tutte le finestre insieme) ---
    if args.agg in ("pooled", "both"):
        pooled_row = {"trial": "== COMPLESSIVO (pooled) ==", "split": f"POOLED_{tag}",
                      "n_finestre": None}
        for ch in channels:
            pr, tr, ps = (np.concatenate(pool[ch][k]) for k in ("pred", "true", "persist"))
            pooled_row["n_finestre"] = len(pr)
            pooled_row.update(channel_columns(ch, pr, tr, ps, metrics, args.baseline))
        rows.append(pooled_row)

    df_new = pd.DataFrame(rows)
    num_cols = [c for c in df_new.columns if c not in ("trial", "split", "n_finestre")]
    df_new[num_cols] = df_new[num_cols].astype(float).round(ROUND_DEC)

    combined = merge_into_master(df_new, args.csv_out, overwrite=args.overwrite)

    pd.set_option("display.width", 260)
    pd.set_option("display.max_columns", None)
    print()
    print(combined.to_string(index=False))

    if args.csv_out:
        combined.to_csv(args.csv_out, index=False)
        print(f"\nMaster aggiornato: {args.csv_out}")
    else:
        print("\n(nessun --csv_out: risultati solo a schermo)")
    print("Unita': " + ", ".join(f"{ch}={CH_UNIT[ch]}" for ch in channels))


if __name__ == "__main__":
    main()