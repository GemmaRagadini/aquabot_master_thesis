"""
Tabella metriche PER TRIAL del modello CONGIUNTO (IM + FM), a hidden incrociato.

FM (diretta) predice i sensori [sensor_diff, current]; IM (inversa) predice il
comando [tail_target]. Testa singola a P passi. Forward a hidden incrociato:
  h_im = GRU_IM(comandi) ; h_fm = GRU_FM(sensori)
  IM.decode(h_im, h_fm, ctx) -> comando ; FM.decode(h_fm, h_im, ctx) -> sensori

Cosa calcolare si sceglie con i FLAG, cosi' ogni run riempie una fetta della
tabella e i risultati si accumulano in un CSV MASTER (merge per chiave
trial+split). Senza flag calcola il MINIMO essenziale (RMSE+nRMSE+R2 a t+1,
baseline persist).

Flag principali:
  --metrics  rmse mae nrmse r2      quali metriche (default: rmse nrmse r2)
  --steps    first avg last         quale posizione d'orizzonte (default: first)
  --baseline persist | none         baseline per skill score (default: persist)
  --channels sensor_diff current cmd (default: tutti)
  --agg      per_trial | pooled | both  righe riassuntive (default: both)
  --all_trials                      includi anche i trial di train
  --overwrite                       ignora il master esistente e riscrivilo

Metriche:
  RMSE, MAE in unita' reali (inverse_transform mono-canale).
  nRMSE = RMSE / std(vero)  (adimensionale; ~0 ottimo, ~1 inutile).
  R2    = 1 - SS_res/SS_tot (1 ottimo, <=0 peggio della media).
  Baseline PERSISTENZA: predizione = valore vero a t. skill = 1 - modello/persist.

Uso:
  python3 .../metrics_per_trial_joint.py --checkpoint .../best_sup_p10.pt
  python3 .../metrics_per_trial_joint.py --checkpoint .../best_sup_p10.pt \
      --metrics rmse nrmse r2 mae --steps first avg last --csv_out master.csv
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

from net.Estimator.model   import build_models
from net.Estimator.dataset import FishJointDataset

FM_CHANNELS = ["sensor_diff", "current"]
IM_CHANNELS = ["cmd"]
ALL_CHANNELS_DEFAULT = FM_CHANNELS + IM_CHANNELS
CH_TO_KEY = {"sensor_diff": "sd", "current": "vf", "cmd": "cmd"}
CH_UNIT   = {"sensor_diff": "unita' sensore (cal)", "current": "mA", "cmd": "rad"}
CH_NET    = {"sensor_diff": "FM", "current": "FM", "cmd": "IM"}

VAL_FRAC   = 0.2
SPLIT_SEED = 42
ROUND_DEC  = 4

METRIC_LABEL = {"rmse": "RMSE", "mae": "MAE", "nrmse": "nRMSE", "r2": "R2"}
STEP_SUF     = {"first": "1", "avg": "avg", "last": "P"}
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
def dims_from_state(sd):
    gru_hidden = sd["gru.weight_hh_l0"].shape[1]
    mlp_hidden = sd["mlp.0.weight"].shape[0]
    return int(gru_hidden), int(mlp_hidden)


def run_trial(dataset, IM, FM, device, trial_idx, batch_size=256):
    """Forward a hidden incrociato su tutte le finestre del trial. Ritorna, in
    scala normalizzata, i P passi predetti/veri e il valore vero a t (persist)."""
    idxs = np.nonzero(dataset.window_trial == trial_idx)[0]
    idxs.sort()
    if len(idxs) == 0:
        return None

    seq_cmd  = dataset.seq_cmd[idxs]
    seq_sens = dataset.seq_sens[idxs]
    ctx      = dataset.context[idxs]
    tgt_cmd  = dataset.tgt_cmd[idxs]      # (n, P, 1)
    tgt_sens = dataset.tgt_sens[idxs]     # (n, P, 2)

    last_cmd = seq_cmd[:, -1, 0].cpu().numpy()   # valore vero a t (persistenza)
    last_sd  = seq_sens[:, -1, 0].cpu().numpy()
    last_vf  = seq_sens[:, -1, 1].cpu().numpy()

    p_cmd_all, p_sens_all = [], []
    with torch.no_grad():
        for s in range(0, len(idxs), batch_size):
            sc = seq_cmd[s:s + batch_size].to(device)
            ss = seq_sens[s:s + batch_size].to(device)
            cx = ctx[s:s + batch_size].to(device)
            h_im = IM.encode(sc)                 # GRU_IM sui comandi
            h_fm = FM.encode(ss)                # GRU_FM sui sensori
            pc = IM.decode(h_im, h_fm, cx)       # (b, P, 1)
            ps = FM.decode(h_fm, h_im, cx)       # (b, P, 2)
            p_cmd_all.append(pc.cpu())
            p_sens_all.append(ps.cpu())

    return {
        "last": {"cmd": last_cmd, "sensor_diff": last_sd, "current": last_vf},
        "tgt":  {"cmd": tgt_cmd.cpu().numpy(),  "sens": tgt_sens.cpu().numpy()},
        "pred": {"cmd": torch.cat(p_cmd_all, 0).numpy(),
                 "sens": torch.cat(p_sens_all, 0).numpy()},
        "P":    tgt_cmd.shape[1],
    }


def channel_steps_real(out, ch, scaler):
    """Per un canale ritorna una lista lunga P: (pred_real, true_real, persist_real)
    a ciascun passo dell'orizzonte, in unita' reali."""
    if ch == "cmd":
        pred = out["pred"]["cmd"][:, :, 0]   # (n, P)
        true = out["tgt"]["cmd"][:, :, 0]
    else:
        ci = FM_CHANNELS.index(ch)
        pred = out["pred"]["sens"][:, :, ci]
        true = out["tgt"]["sens"][:, :, ci]
    last = out["last"][ch]                    # (n,) valore vero a t

    def inv(a):
        return scaler.inverse_transform(np.asarray(a).reshape(-1, 1)).ravel()

    P = pred.shape[1]
    persist_real = inv(last)                  # stesso per ogni passo
    steps = []
    for k in range(P):
        steps.append((inv(pred[:, k]), inv(true[:, k]), persist_real))
    return steps


# ------------------------------- colonne per canale -------------------------------
def channel_columns(ch, steps_real, metrics, steps, baseline):
    """Costruisce le colonne per un canale secondo i flag selezionati.
    steps_real: lista P di (pred_real, true_real, persist_real)."""
    P = len(steps_real)
    idx_of = {"first": [0], "last": [P - 1], "avg": list(range(P))}
    cols = {f"{ch}_net": CH_NET[ch], f"{ch}_P": P}
    skill_metrics = [m for m in metrics if m in SKILL_METRICS] if baseline == "persist" else []

    for st in steps:
        suf = STEP_SUF[st]
        ks = idx_of[st]
        # metriche del modello (media sui passi selezionati; per 'first'/'last' un solo k)
        for mt in metrics:
            vals = [METRIC_FUNCS[mt](steps_real[k][0], steps_real[k][1]) for k in ks]
            cols[f"{ch}_{METRIC_LABEL[mt]}_{suf}"] = float(np.nanmean(vals))
        # baseline persistenza + skill (solo rmse/mae)
        for mt in skill_metrics:
            pv = [METRIC_FUNCS[mt](steps_real[k][2], steps_real[k][1]) for k in ks]
            cols[f"{ch}_{METRIC_LABEL[mt]}_persist_{suf}"] = float(np.nanmean(pv))
            sk = []
            for k in ks:
                mm = METRIC_FUNCS[mt](steps_real[k][0], steps_real[k][1])
                pp = METRIC_FUNCS[mt](steps_real[k][2], steps_real[k][1])
                sk.append(1.0 - mm / pp if pp > 0 else float("nan"))
            cols[f"{ch}_skill_{mt}_{suf}"] = float(np.nanmean(sk))
    return cols


def column_order(channels, metrics, steps, baseline):
    """Ordine deterministico delle colonne metriche (le nuove di questo run)."""
    order = []
    skill_metrics = [m for m in metrics if m in SKILL_METRICS] if baseline == "persist" else []
    for ch in channels:
        order += [f"{ch}_net", f"{ch}_P"]
        for st in steps:
            suf = STEP_SUF[st]
            for mt in metrics:
                order.append(f"{ch}_{METRIC_LABEL[mt]}_{suf}")
            for mt in skill_metrics:
                order.append(f"{ch}_{METRIC_LABEL[mt]}_persist_{suf}")
                order.append(f"{ch}_skill_{mt}_{suf}")
    return order


# ------------------------------- merge nel master -------------------------------
def row_sort_key(trial, split):
    if str(trial).startswith("== MEDIA"):
        return (3, str(trial))
    if str(trial).startswith("== COMPLESSIVO"):
        return (4, str(trial))
    return ({"val": 0, "train": 1}.get(split, 2), str(trial))


def merge_into_master(df_new, master_path, overwrite=False):
    """Unisce df_new nel CSV master per chiave (trial, split): le colonne calcolate
    in questo run sovrascrivono/aggiungono quelle vecchie, le altre restano;
    righe nuove vengono aggiunte. Ritorna il DataFrame combinato."""
    key = ["trial", "split"]
    if overwrite or not os.path.exists(master_path):
        combined = df_new.copy()
    else:
        df_old = pd.read_csv(master_path)
        old_i = df_old.set_index(key)
        new_i = df_new.set_index(key)
        # new vince dove ha valori (non-NA); vecchie colonne/righe conservate
        combined = new_i.combine_first(old_i).reset_index()

    # ordine colonne: lead + nuove (in ordine) + eventuali vecchie rimaste
    lead = ["trial", "split", "n_finestre"]
    new_cols = [c for c in df_new.columns if c not in lead]
    rest = [c for c in combined.columns if c not in lead + new_cols]
    ordered = [c for c in lead if c in combined.columns] + \
              [c for c in new_cols if c in combined.columns] + rest
    combined = combined[ordered]

    # ordine righe: val, train, poi MEDIA, poi COMPLESSIVO
    combined = combined.sort_values(
        by=key, key=lambda col: col if col.name not in key else col,
        kind="stable")
    combined["_sk"] = [row_sort_key(t, s) for t, s in
                       zip(combined["trial"], combined["split"])]
    combined = combined.sort_values("_sk").drop(columns="_sk").reset_index(drop=True)
    return combined


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--checkpoint", default=os.path.join(SCRIPT_DIR, "checkpoints_joint", "best.pt"))
    ap.add_argument("--dataset_dir", default=os.path.join(REPO_ROOT, "src", "net", "dataset"))
    ap.add_argument("--scaler_path", default=os.path.join(REPO_ROOT, "src", "net", "scaler", "scalers_joint.pkl"),
                    help="normalizzatore condiviso: DEVE essere lo stesso del training")
    ap.add_argument("--device", default="cuda" if torch.cuda.is_available() else "cpu")
    ap.add_argument("--all_trials", action="store_true",
                    help="includi anche i trial di train (default: solo validation)")
    ap.add_argument("--csv_out", default=os.path.join(SCRIPT_DIR, "metrics_master.csv"),
                    help="CSV master: viene aggiornato per merge a ogni run")
    ap.add_argument("--overwrite", action="store_true",
                    help="ignora il master esistente e riscrivilo da zero")
    ap.add_argument("--p", type=int, default=None,
                    help="orizzonte P del dataset. Default: il P del checkpoint.")

    # --- selezione di COSA calcolare ---
    ap.add_argument("--metrics", nargs="+", default=["rmse", "nrmse", "r2"],
                    choices=["rmse", "mae", "nrmse", "r2"],
                    help="quali metriche (default: rmse nrmse r2).")
    ap.add_argument("--steps", nargs="+", default=["first"],
                    choices=["first", "avg", "last"],
                    help="posizione d'orizzonte: first=t+1, avg=media su P, last=t+P.")
    ap.add_argument("--baseline", default="persist", choices=["persist", "none"],
                    help="baseline per lo skill score (default: persist).")
    ap.add_argument("--channels", nargs="+", default=ALL_CHANNELS_DEFAULT,
                    choices=ALL_CHANNELS_DEFAULT,
                    help="quali canali (default: tutti).")
    ap.add_argument("--agg", default="both", choices=["per_trial", "pooled", "both"],
                    help="righe riassuntive da aggiungere (default: both).")
    args = ap.parse_args()

    device = torch.device(args.device)

    ckpt = torch.load(args.checkpoint, map_location=device, weights_only=False)
    P_ckpt = ckpt.get("P", 1)
    P = args.p if args.p is not None else P_ckpt
    if args.p is not None and args.p != P_ckpt:
        print(f"[avviso] --p={args.p} diverso dal P del checkpoint ({P_ckpt}).")
    print(f"P usato per il dataset: {P} (checkpoint: {P_ckpt})")

    # 'first'/'last' coincidono con P=1: dedup silenzioso mantenendo l'ordine
    steps = list(dict.fromkeys(args.steps))
    if P == 1 and any(s in steps for s in ("avg", "last")):
        print("[avviso] P=1: 'avg' e 'last' coincidono con 'first'.", file=sys.stderr)

    dataset = FishJointDataset(args.dataset_dir, p=P, scaler_path=args.scaler_path)
    _, val_ds = dataset.split_by_trial(val_frac=VAL_FRAC, seed=SPLIT_SEED)
    val_trials = set(int(i) for i in np.unique(
        dataset.window_trial[np.asarray(val_ds.indices)]))

    ctx_static = ckpt.get("ctx_static", int(dataset.context.shape[-1]))
    gh_im, mh_im = dims_from_state(ckpt["im_state"])
    gh_fm, mh_fm = dims_from_state(ckpt["fm_state"])
    print(f"Dimensioni dal checkpoint: IM gru={gh_im} mlp={mh_im} | "
          f"FM gru={gh_fm} mlp={mh_fm} | P={P} | ctx_static={ctx_static}")

    ds_ctx = int(dataset.context.shape[-1])
    if ctx_static != ds_ctx:
        raise ValueError(f"MISMATCH contesto: checkpoint ctx_static={ctx_static} ma "
                         f"il dataset ne produce {ds_ctx}.")

    IM, FM = build_models(gru_hidden_im=gh_im, mlp_hidden_im=mh_im,
                          gru_hidden_fm=gh_fm, mlp_hidden_fm=mh_fm,
                          p=P, ctx_static=ctx_static)
    IM.load_state_dict(ckpt["im_state"]); IM.to(device).eval()
    FM.load_state_dict(ckpt["fm_state"]); FM.to(device).eval()

    channels = list(dict.fromkeys(args.channels))
    print(f"Metriche: {args.metrics} | steps: {steps} | baseline: {args.baseline} | "
          f"canali: {channels} | agg: {args.agg}")

    trials = list(range(len(dataset.trial_names))) if args.all_trials else sorted(val_trials)
    print(f"trial considerati: {len(trials)} "
          f"({'tutti' if args.all_trials else 'solo validation'})")

    # accumulatori per il POOLED: per canale e passo, gli array reali di tutti i trial
    pool = {ch: None for ch in channels}   # ch -> lista P di dict{pred,true,persist}

    rows = []
    per_trial_metric_rows = []   # solo colonne metriche, per la MEDIA per-trial
    for ti in trials:
        out = run_trial(dataset, IM, FM, device, ti)
        if out is None:
            continue
        split = "val" if ti in val_trials else "train"
        n_win = len(out["tgt"]["cmd"])

        row = {"trial": dataset.trial_names[ti], "split": split, "n_finestre": n_win}
        metric_only = {}
        for ch in channels:
            sc = dataset.scalers[CH_TO_KEY[ch]]
            steps_real = channel_steps_real(out, ch, sc)
            cols = channel_columns(ch, steps_real, args.metrics, steps, args.baseline)
            row.update(cols)
            metric_only.update({k: v for k, v in cols.items()
                                if not k.endswith("_net")})  # _P resta (int)

            # accumulo pooled
            if pool[ch] is None:
                pool[ch] = [{"pred": [], "true": [], "persist": []}
                            for _ in range(len(steps_real))]
            for k, (pr, tr, ps) in enumerate(steps_real):
                pool[ch][k]["pred"].append(pr)
                pool[ch][k]["true"].append(tr)
                pool[ch][k]["persist"].append(ps)

        rows.append(row)
        per_trial_metric_rows.append(metric_only)

    if not rows:
        print("Nessun trial elaborato.", file=sys.stderr)
        sys.exit(1)

    # --- riga MEDIA per-trial (ogni trial pesa uguale) ---
    if args.agg in ("per_trial", "both"):
        mean_row = {"trial": "== MEDIA per-trial ==",
                    "split": "MEDIA_val" if not args.all_trials else "MEDIA_all",
                    "n_finestre": len(per_trial_metric_rows)}
        allkeys = set().union(*[set(r) for r in per_trial_metric_rows])
        for k in allkeys:
            vals = [r[k] for r in per_trial_metric_rows if k in r]
            if all(isinstance(v, (int, np.integer)) for v in vals):   # es. _P
                mean_row[k] = int(vals[0])
            else:
                mean_row[k] = float(np.nanmean(np.asarray(vals, dtype=float)))
        # ripristina i _net (stringhe) persi dal metric_only
        for ch in channels:
            mean_row[f"{ch}_net"] = CH_NET[ch]
        rows.append(mean_row)

    # --- riga POOLED (tutte le finestre insieme) ---
    if args.agg in ("pooled", "both"):
        pooled_row = {"trial": "== COMPLESSIVO (pooled) ==",
                      "split": "POOLED_val" if not args.all_trials else "POOLED_all",
                      "n_finestre": None}
        n_pool = 0
        for ch in channels:
            steps_real = [(np.concatenate(pool[ch][k]["pred"]),
                           np.concatenate(pool[ch][k]["true"]),
                           np.concatenate(pool[ch][k]["persist"]))
                          for k in range(len(pool[ch]))]
            n_pool = len(steps_real[0][0])
            cols = channel_columns(ch, steps_real, args.metrics, steps, args.baseline)
            pooled_row.update(cols)
        pooled_row["n_finestre"] = n_pool
        rows.append(pooled_row)

    df_new = pd.DataFrame(rows)

    # arrotonda le colonne float (lascia _net stringa e _P int)
    num_cols = [c for c in df_new.columns
                if c not in ("trial", "split", "n_finestre")
                and not c.endswith("_net") and not c.endswith("_P")]
    df_new[num_cols] = df_new[num_cols].astype(float).round(ROUND_DEC)

    combined = merge_into_master(df_new, args.csv_out, overwrite=args.overwrite)

    pd.set_option("display.width", 260)
    pd.set_option("display.max_columns", None)
    print()
    print(combined.to_string(index=False))

    combined.to_csv(args.csv_out, index=False)
    print(f"\nMaster aggiornato: {args.csv_out}")
    print("Unita': " + ", ".join(f"{ch}={CH_UNIT[ch]}" for ch in channels))


if __name__ == "__main__":
    main()