"""
Tabella RMSE e MAE PER OGNI TRIAL, per ogni canale, in unita' reali

Le metriche di DEFAULT sono calcolate solo sui trial di VALIDATION (metriche
oneste: episodi mai visti dal modello). Con --all_trials si includono anche i
trial di train, etichettati nella colonna 'split'.

Uso:

  python3 src/net/SensorEstimator/checkpoints/metrics_per_trial.py --checkpoint src/net/SensorEstimator/checkpoints/best.pt
  python3 src/net/SensorEstimator/checkpoints/metrics_per_trial.py --checkpoint src/net/SensorEstimator/checkpoints/best.pt --all_trials
  python3 src/net/SensorEstimator/checkpoints/metrics_per_trial.py --checkpoint src/net/SensorEstimator/checkpoints/best.pt --csv_out src/net/SensorEstimator/checkpoints/metrics.csv
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

from net.SensorEstimator.model import FishSensorEstimator
from net.SensorEstimator.dataset import FishDataset

# Ordine canonico dei canali. DEVE combaciare con l'ordine con cui il dataset
# impila i target in _build_windows: attualmente [sensor_diff, current], con
# [sensor_diff, sensor_mean, current] se sensor_mean viene riattivato (N_OUTPUTS=3).
CHANNELS_ALL = ["sensor_diff", "sensor_mean", "current"]
CHANNELS_2   = ["sensor_diff", "current"]
CH_TO_KEY    = {"sensor_diff": "sd", "sensor_mean": "sm", "current": "vf"}
CH_UNIT      = {"sensor_diff": "unita' sensore (cal)",
                "sensor_mean": "unita' sensore (cal)",
                "current": "mA"}

VAL_FRAC   = 0.2
SPLIT_SEED = 42
ROUND_DEC  = 4   # decimali salvati nel CSV


def dims_from_state_dict(sd):
    """Ricostruisce gru_hidden e mlp_hidden dalle shape dei pesi salvati,
    come in channel_loss.py: niente flag da passare a mano."""
    gru_hidden = sd["gru.weight_hh_l0"].shape[1]
    mlp_hidden = sd["mlp.0.weight"].shape[0]
    return int(gru_hidden), int(mlp_hidden)


def _real_err(pred_norm, true_norm, scaler):
    """Errori (pred - true) in unita' reali, appiattiti in 1D.
    Lo scaler e' mono-canale, quindi il reshape(-1,1) e' lecito e l'ordine
    degli elementi non influisce sul risultato."""
    pred = scaler.inverse_transform(np.asarray(pred_norm).reshape(-1, 1)).ravel()
    true = scaler.inverse_transform(np.asarray(true_norm).reshape(-1, 1)).ravel()
    return pred - true


def rmse_mae(err):
    """RMSE e MAE da un array di errori gia' in unita' reali."""
    err = np.asarray(err)
    rmse = float(np.sqrt(np.mean(err ** 2)))
    mae  = float(np.mean(np.abs(err)))
    return rmse, mae


def run_trial(dataset, model, device, trial_idx, batch_size=256):
    """Fa girare il modello su tutte le finestre di un trial e restituisce
    target/predizioni per entrambe le teste (in scala normalizzata)."""
    idxs = np.nonzero(dataset.window_trial == trial_idx)[0]
    idxs.sort()  # le finestre di un trial sono contigue e in ordine temporale
    if len(idxs) == 0:
        return None

    seq    = dataset.sequences[idxs]
    t_hist = dataset.targets_history[idxs]   # (n, h, nc)
    t_fut  = dataset.targets_future[idxs]    # (n, nc)

    ph, pf = [], []
    with torch.no_grad():
        for s in range(0, len(idxs), batch_size):
            chunk = seq[s:s + batch_size].to(device)
            pred_history, pred_future, _ = model(chunk)
            ph.append(pred_history.cpu())
            pf.append(pred_future.cpu())
    return t_hist, torch.cat(ph), t_fut, torch.cat(pf)


def build_wide_row(trial_name, split, n_win, per_channel):
    """Costruisce una riga 'wide': campi anagrafici + una colonna per ogni
    (canale, metrica). per_channel[ch] = dict con rmse/mae hist/fut,
    piu' persistenza e percentuali.

    Colonne per canale (testa future, la piu' importante):
      {ch}_RMSE_hist, {ch}_MAE_hist      -> errore testa history
      {ch}_RMSE_fut,  {ch}_MAE_fut       -> errore testa future
      {ch}_RMSE_persist                  -> baseline persistenza RMSE (predici t-1)
      {ch}_MAE_persist                   -> baseline persistenza MAE
      {ch}_ppk                           -> ampiezza picco-picco del segnale reale
      {ch}_RMSE_fut_pct                  -> RMSE_fut / ppk * 100
      {ch}_persist_pct                   -> RMSE_persist / ppk * 100
      {ch}_MAE_fut_pct                   -> MAE_fut / ppk * 100
      {ch}_MAE_persist_pct               -> MAE_persist / ppk * 100
      {ch}_skill_rmse                    -> 1 - RMSE_fut/RMSE_persist
                                            (>0 = batte la persistenza)
      {ch}_skill_mae                     -> 1 - MAE_fut/MAE_persist
                                            (piu' robusto agli outlier)
    """
    row = {"trial": trial_name, "split": split, "n_finestre": n_win}
    for ch, m in per_channel.items():
        row[f"{ch}_RMSE_hist"]       = m["rmse_h"]
        row[f"{ch}_MAE_hist"]        = m["mae_h"]
        row[f"{ch}_RMSE_fut"]        = m["rmse_f"]
        row[f"{ch}_MAE_fut"]         = m["mae_f"]
        row[f"{ch}_RMSE_persist"]    = m["rmse_persist"]
        row[f"{ch}_MAE_persist"]     = m["mae_persist"]
        row[f"{ch}_ppk"]             = m["ppk"]
        row[f"{ch}_RMSE_fut_pct"]    = m["rmse_fut_pct"]
        row[f"{ch}_persist_pct"]     = m["persist_pct"]
        row[f"{ch}_MAE_fut_pct"]     = m["mae_fut_pct"]
        row[f"{ch}_MAE_persist_pct"] = m["mae_persist_pct"]
        row[f"{ch}_skill_rmse"]      = m["skill_rmse"]
        row[f"{ch}_skill_mae"]       = m["skill_mae"]
    return row


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--checkpoint", default=os.path.join(SCRIPT_DIR, "best.pt"))
    ap.add_argument("--dataset_dir", default=os.path.join(REPO_ROOT, "src", "net", "dataset"))
    ap.add_argument("--scaler_path", default=os.path.join(REPO_ROOT, "src", "net", "scaler", "scalers.pkl"),
                    help="normalizzatore da riusare: DEVE essere lo stesso del training")
    ap.add_argument("--device", default="cuda" if torch.cuda.is_available() else "cpu")
    ap.add_argument("--all_trials", action="store_true",
                    help="includi anche i trial di train (default: solo validation)")
    ap.add_argument("--csv_out", default=os.path.join(SCRIPT_DIR, "metrics_per_trial.csv"))
    args = ap.parse_args()

    device = torch.device(args.device)

    # stessa normalizzazione e stesso split del training
    dataset = FishDataset(args.dataset_dir, scaler_path=args.scaler_path)
    _, val_ds = dataset.split_by_trial(val_frac=VAL_FRAC, seed=SPLIT_SEED)
    val_trials = set(int(i) for i in np.unique(
        dataset.window_trial[np.asarray(val_ds.indices)]))

    ckpt = torch.load(args.checkpoint, map_location=device)
    state = ckpt["model_state"]
    input_size = ckpt.get("input_size", dataset.sequences.shape[-1])
    gru_hidden, mlp_hidden = dims_from_state_dict(state)
    print(f"input_size={input_size} | gru_hidden={gru_hidden} | mlp_hidden={mlp_hidden} "
          f"(letti dal checkpoint)")

    # --- ASSERT DI COERENZA: evitano numeri sbagliati silenziosi ---
    ds_input = int(dataset.sequences.shape[-1])
    if input_size != ds_input:
        raise ValueError(
            f"MISMATCH input feature: il checkpoint ha input_size={input_size} ma il "
            f"dataset ne produce {ds_input}. Le sequenze di input non sono compatibili "
            f"col modello salvato (controlla N_INPUT_FEATURES nel dataset e le feature "
            f"attive in _build_windows). I risultati sarebbero privi di senso."
        )

    model = FishSensorEstimator(input_size=input_size,
                                gru_hidden=gru_hidden,
                                mlp_hidden=mlp_hidden,
                                h=dataset.h).to(device)
    model.load_state_dict(state)
    model.eval()

    nc = dataset.targets_future.shape[-1]
    CHANNELS = CHANNELS_2 if nc == 2 else CHANNELS_ALL

    # il modello deve predire esattamente nc canali, altrimenti le colonne si
    # disallineano rispetto ai target del dataset
    out_feat = model.head_future.out_features
    if out_feat != nc:
        raise ValueError(
            f"MISMATCH canali: il modello predice {out_feat} canali ma il dataset "
            f"ne ha {nc}. Probabilmente N_OUTPUTS in model.py non e' allineato ai "
            f"target attivi in dataset._build_windows."
        )
    print(f"canali predetti: {nc} -> {CHANNELS}")

    if args.all_trials:
        trials = list(range(len(dataset.trial_names)))
    else:
        trials = sorted(val_trials)
    print(f"trial considerati: {len(trials)} "
          f"({'tutti' if args.all_trials else 'solo validation'})")

    # accumulatori per il POOLED: errori reali di TUTTE le finestre mostrate,
    # per canale e per testa. Il pooled e' una metrica unica pesata per lunghezza
    # (non la media delle medie per-trial). Accumulo anche l'errore di
    # persistenza e i valori veri (per il picco-picco pooled).
    err_pool = {ch: {"hist": [], "fut": [], "persist": [], "true_f": []}
                for ch in CHANNELS}

    rows = []
    for ti in trials:
        out = run_trial(dataset, model, device, ti)
        if out is None:
            continue
        t_hist, p_hist, t_fut, p_fut = out
        split = "val" if ti in val_trials else "train"

        per_channel = {}
        for ci, ch in enumerate(CHANNELS):
            sc = dataset.scalers[CH_TO_KEY[ch]]

            e_h = _real_err(p_hist[:, :, ci].numpy(), t_hist[:, :, ci].numpy(), sc)
            e_f = _real_err(p_fut[:, ci].numpy(),     t_fut[:, ci].numpy(),     sc)

            rmse_h, mae_h = rmse_mae(e_h)
            rmse_f, mae_f = rmse_mae(e_f)

            # --- baseline PERSISTENZA (testa future) ---
            # predizione a t+1 = valore vero a t = ultimo istante della history.
            # e' l'errore che farebbe "il modello che non fa nulla".
            true_t   = t_hist[:, -1, ci].numpy()   # valore vero a t (norm)
            true_tp1 = t_fut[:, ci].numpy()         # valore vero a t+1 (norm)
            e_persist = _real_err(true_t, true_tp1, sc)  # (t - (t+1)) in reale
            rmse_persist, mae_persist = rmse_mae(e_persist)

            # --- ampiezza picco-picco del segnale REALE del trial ---
            # uso i veri a t+1 in unita' reali come campione del segnale
            true_real = sc.inverse_transform(true_tp1.reshape(-1, 1)).ravel()
            ppk = float(true_real.max() - true_real.min())

            rmse_fut_pct    = 100.0 * rmse_f / ppk if ppk > 0 else float("nan")
            persist_pct     = 100.0 * rmse_persist / ppk if ppk > 0 else float("nan")
            mae_fut_pct     = 100.0 * mae_f / ppk if ppk > 0 else float("nan")
            mae_persist_pct = 100.0 * mae_persist / ppk if ppk > 0 else float("nan")
            # skill score: quanto il modello migliora sulla persistenza.
            # >0 => batte la baseline; =0 => equivale; <0 => peggio.
            # skill_rmse penalizza gli errori grossi; skill_mae e' piu' robusto
            # agli outlier.
            skill_rmse = (1.0 - rmse_f / rmse_persist) if rmse_persist > 0 else float("nan")
            skill_mae  = (1.0 - mae_f / mae_persist) if mae_persist > 0 else float("nan")

            per_channel[ch] = {"rmse_h": rmse_h, "mae_h": mae_h,
                               "rmse_f": rmse_f, "mae_f": mae_f,
                               "rmse_persist": rmse_persist,
                               "mae_persist": mae_persist, "ppk": ppk,
                               "rmse_fut_pct": rmse_fut_pct,
                               "persist_pct": persist_pct,
                               "mae_fut_pct": mae_fut_pct,
                               "mae_persist_pct": mae_persist_pct,
                               "skill_rmse": skill_rmse, "skill_mae": skill_mae}

            err_pool[ch]["hist"].append(e_h)
            err_pool[ch]["fut"].append(e_f)
            err_pool[ch]["persist"].append(e_persist)
            err_pool[ch]["true_f"].append(true_real)

        rows.append(build_wide_row(dataset.trial_names[ti], split,
                                   len(t_fut), per_channel))

    # ordina: prima val, poi train, per nome trial dentro ogni gruppo
    split_rank = {"val": 0, "train": 1}
    rows.sort(key=lambda r: (split_rank.get(r["split"], 2), r["trial"]))

    # --- riga POOLED (tutte le finestre mostrate insieme, pesate per lunghezza) ---
    # NB: il picco-picco pooled e' calcolato sull'intero segnale reale messo
    # insieme (min/max globali sui trial mostrati), non come media dei ppk.
    pooled_channel = {}
    n_pool = 0
    for ch in CHANNELS:
        e_h  = np.concatenate(err_pool[ch]["hist"])
        e_f  = np.concatenate(err_pool[ch]["fut"])
        e_p  = np.concatenate(err_pool[ch]["persist"])
        true = np.concatenate(err_pool[ch]["true_f"])
        rmse_h, mae_h = rmse_mae(e_h)
        rmse_f, mae_f = rmse_mae(e_f)
        rmse_persist, mae_persist = rmse_mae(e_p)
        ppk = float(true.max() - true.min())
        rmse_fut_pct    = 100.0 * rmse_f / ppk if ppk > 0 else float("nan")
        persist_pct     = 100.0 * rmse_persist / ppk if ppk > 0 else float("nan")
        mae_fut_pct     = 100.0 * mae_f / ppk if ppk > 0 else float("nan")
        mae_persist_pct = 100.0 * mae_persist / ppk if ppk > 0 else float("nan")
        skill_rmse = (1.0 - rmse_f / rmse_persist) if rmse_persist > 0 else float("nan")
        skill_mae  = (1.0 - mae_f / mae_persist) if mae_persist > 0 else float("nan")
        pooled_channel[ch] = {"rmse_h": rmse_h, "mae_h": mae_h,
                              "rmse_f": rmse_f, "mae_f": mae_f,
                              "rmse_persist": rmse_persist,
                              "mae_persist": mae_persist, "ppk": ppk,
                              "rmse_fut_pct": rmse_fut_pct,
                              "persist_pct": persist_pct,
                              "mae_fut_pct": mae_fut_pct,
                              "mae_persist_pct": mae_persist_pct,
                              "skill_rmse": skill_rmse, "skill_mae": skill_mae}
        n_pool = len(e_f)
    pooled_split = "POOLED_val" if not args.all_trials else "POOLED_all"
    pooled_row = build_wide_row("== COMPLESSIVO (pooled) ==", pooled_split,
                                n_pool, pooled_channel)

    df = pd.DataFrame(rows)
    pooled_df = pd.DataFrame([pooled_row])
    df_out = pd.concat([df, pooled_df], ignore_index=True)

    # ordine colonne: anagrafica, poi i blocchi per canale
    lead = ["trial", "split", "n_finestre"]
    metric_cols = []
    for ch in CHANNELS:
        metric_cols += [f"{ch}_RMSE_hist", f"{ch}_MAE_hist",
                        f"{ch}_RMSE_fut",  f"{ch}_MAE_fut",
                        f"{ch}_RMSE_persist", f"{ch}_MAE_persist", f"{ch}_ppk",
                        f"{ch}_RMSE_fut_pct", f"{ch}_persist_pct",
                        f"{ch}_MAE_fut_pct", f"{ch}_MAE_persist_pct",
                        f"{ch}_skill_rmse", f"{ch}_skill_mae"]
    df_out = df_out[lead + metric_cols]

    # arrotonda (vale SIA per lo stampato SIA per il CSV salvato)
    df_out[metric_cols] = df_out[metric_cols].round(ROUND_DEC)

    pd.set_option("display.width", 240)
    pd.set_option("display.max_columns", None)
    print()
    print(df_out.to_string(index=False))

    df_out.to_csv(args.csv_out, index=False)
    print(f"\nTabella salvata in {args.csv_out}")
    print("Unita': " + ", ".join(f"{ch}={CH_UNIT[ch]}" for ch in CHANNELS))

    # --- verdetto sintetico sul POOLED ---
    print("\n=== Verdetto (pooled, testa future) ===")
    for ch in CHANNELS:
        m = pooled_channel[ch]
        verdict = ("batte la persistenza" if m["skill_rmse"] > 0.05 else
                   "~come la persistenza" if m["skill_rmse"] > -0.05 else
                   "PEGGIO della persistenza")
        print(f"  {ch:<12} RMSE {m['rmse_f']:.2f} ({m['rmse_fut_pct']:.1f}% ppk) | "
              f"MAE {m['mae_f']:.2f} ({m['mae_fut_pct']:.1f}% ppk) | "
              f"persist RMSE {m['rmse_persist']:.2f} MAE {m['mae_persist']:.2f} | "
              f"skill_rmse {m['skill_rmse']:+.2f} | skill_mae {m['skill_mae']:+.2f} "
              f"-> {verdict}")


if __name__ == "__main__":
    main()