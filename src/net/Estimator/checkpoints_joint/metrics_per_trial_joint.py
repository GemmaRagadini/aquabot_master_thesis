"""
Tabelle RMSE e MAE PER OGNI TRIAL, per ogni canale, in unita' reali —
versione per il modello CONGIUNTO (IM + FM).

Differenze rispetto alla versione vecchia (FishSensorEstimator):
  - Due reti, non una: FM (diretta) predice i sensori [sensor_diff, current],
    IM (inversa) predice il comando [tail_target]. Testa SINGOLA a P passi
    (niente piu' teste history/future): si valuta il PRIMO passo predetto.
  - Ingresso condiviso [C, S] concatenato sui canali (b, H, 3) + contesto (b, 4).
  - Dataset FishJointDataset, scaler unico condiviso.

Metriche (identiche alla versione vecchia, sono il valore dello script):
  - RMSE / MAE in unita' fisiche (inverse_transform con lo scaler mono-canale).
  - Baseline PERSISTENZA: predizione a t+1 = valore vero a t (ultimo istante di
    input). E' l'errore del "modello che non fa nulla".
  - Skill score: 1 - RMSE_modello / RMSE_persist  (>0 => batte la persistenza).
  - Percentuali su picco-picco del segnale reale del trial.
  - POOLED pesato per lunghezza (non media delle medie).

Metriche di DEFAULT sui soli trial di VALIDATION (episodi mai visti). Con
--all_trials si includono anche i trial di train, etichettati in 'split'.

Uso:
python3 src/net/Estimator/checkpoints_joint/metrics_per_trial_joint.py 
--checkpoint src/net/Estimator/checkpoints_joint/best_P1.pt

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

# flat import (come train_joint / plot_prediction_joint).
from net.Estimator.model   import build_models
from net.Estimator.dataset import FishJointDataset

# Canali per rete. FM predice i sensori (2 canali, nell'ordine con cui il
# dataset impila tgt_sens: [sensor_diff, current]); IM predice il comando.
FM_CHANNELS = ["sensor_diff", "current"]
IM_CHANNELS = ["cmd"]
CH_TO_KEY = {"sensor_diff": "sd", "current": "vf", "cmd": "cmd"}
CH_UNIT   = {"sensor_diff": "unita' sensore (cal)",
             "current": "mA",
             "cmd": "rad"}
# a quale rete appartiene ogni canale (per etichettare le tabelle)
CH_NET = {"sensor_diff": "FM", "current": "FM", "cmd": "IM"}

VAL_FRAC   = 0.2
SPLIT_SEED = 42
ROUND_DEC  = 4


def dims_from_state(sd):
    """gru_hidden e mlp_hidden dalle shape dei pesi (non salvate nel checkpoint),
    come in plot_prediction_joint / closed_loop_test."""
    gru_hidden = sd["gru.weight_hh_l0"].shape[1]
    mlp_hidden = sd["mlp.0.weight"].shape[0]
    return int(gru_hidden), int(mlp_hidden)


def _real_err(pred_norm, true_norm, scaler):
    """Errori (pred - true) in unita' reali, appiattiti in 1D. Lo scaler e'
    mono-canale, quindi reshape(-1,1) e' lecito."""
    pred = scaler.inverse_transform(np.asarray(pred_norm).reshape(-1, 1)).ravel()
    true = scaler.inverse_transform(np.asarray(true_norm).reshape(-1, 1)).ravel()
    return pred - true


def rmse_mae(err):
    err = np.asarray(err)
    rmse = float(np.sqrt(np.mean(err ** 2)))
    mae  = float(np.mean(np.abs(err)))
    return rmse, mae


def run_trial(dataset, IM, FM, device, trial_idx, batch_size=256):
    """Fa girare ENTRAMBE le reti su tutte le finestre (contigue, ordinate) di un
    trial. Ritorna, in scala normalizzata e al PRIMO passo predetto (P index 0):

      last_cmd  (n,)     valore vero comando a t  (per la persistenza IM)
      last_sd   (n,)     valore vero sensor_diff a t
      last_vf   (n,)     valore vero current a t
      tgt_cmd   (n, 1)   target comando a t+1
      pred_cmd  (n, 1)   predizione IM a t+1
      tgt_sens  (n, 2)   target sensori a t+1
      pred_sens (n, 2)   predizione FM a t+1
    """
    idxs = np.nonzero(dataset.window_trial == trial_idx)[0]
    idxs.sort()
    if len(idxs) == 0:
        return None

    seq_cmd  = dataset.seq_cmd[idxs]
    seq_sens = dataset.seq_sens[idxs]
    ctx      = dataset.context[idxs]
    tgt_cmd  = dataset.tgt_cmd[idxs]      # (n, P, 1)
    tgt_sens = dataset.tgt_sens[idxs]     # (n, P, 2)

    # valore vero a t = ultimo istante della history (per la persistenza)
    last_cmd = seq_cmd[:, -1, 0].cpu().numpy()          # (n,)
    last_sd  = seq_sens[:, -1, 0].cpu().numpy()          # (n,)
    last_vf  = seq_sens[:, -1, 1].cpu().numpy()          # (n,)

    p_cmd_all, p_sens_all = [], []
    with torch.no_grad():
        for s in range(0, len(idxs), batch_size):
            sc = seq_cmd[s:s + batch_size].to(device)
            ss = seq_sens[s:s + batch_size].to(device)
            cx = ctx[s:s + batch_size].to(device)
            seq = torch.cat([sc, ss], dim=-1)            # (b, H, 3)
            pc, _ = IM(seq, cx)                          # (b, P, 1)
            ps, _ = FM(seq, cx)                          # (b, P, 2)
            p_cmd_all.append(pc.cpu())
            p_sens_all.append(ps.cpu())

    # tengo TUTTI i P passi (b, P, C): le metriche calcolano sia il primo passo
    # sia la media per-passo.
    pred_cmd  = torch.cat(p_cmd_all,  dim=0).numpy()   # (n, P, 1)
    pred_sens = torch.cat(p_sens_all, dim=0).numpy()   # (n, P, 2)
    tgt_cmd_P  = tgt_cmd.cpu().numpy()                 # (n, P, 1)
    tgt_sens_P = tgt_sens.cpu().numpy()               # (n, P, 2)

    return {
        "last": {"cmd": last_cmd, "sensor_diff": last_sd, "current": last_vf},
        "tgt":  {"cmd": tgt_cmd_P, "sens": tgt_sens_P},   # (n, P, C)
        "pred": {"cmd": pred_cmd,  "sens": pred_sens},    # (n, P, C)
        "P":    pred_cmd.shape[1],
    }


def channel_arrays(out, ch):
    """Estrae (pred_norm, true_norm, last_true_norm) per un canale, gestendo
    l'indicizzazione FM (2 canali) vs IM (1 canale).
    pred e true hanno shape (n, P): tutti i passi dell'orizzonte."""
    if ch == "cmd":
        pred = out["pred"]["cmd"][:, :, 0]   # (n, P)
        true = out["tgt"]["cmd"][:, :, 0]    # (n, P)
    else:
        ci = FM_CHANNELS.index(ch)           # 0=sensor_diff, 1=current
        pred = out["pred"]["sens"][:, :, ci]
        true = out["tgt"]["sens"][:, :, ci]
    last = out["last"][ch]                    # (n,) valore vero a t
    return pred, true, last


def channel_metrics(pred_norm, true_norm, last_norm, scaler):
    """RMSE/MAE reali + persistenza + percentuali su picco-picco + skill.

    pred_norm, true_norm: (n, P). Calcola DUE set di metriche:
      - '_1' : solo il primo passo predetto (t+1), come la versione one-step.
      - '_avg': media delle metriche per-passo sull'orizzonte P
                (RMSE del passo k, mediata su k). Con P=1 coincide col primo.

    La persistenza usa sempre 'valore vero a t' (last) come predizione per ogni
    passo: baseline naive coerente con la versione vecchia. Diventa piu' severa
    per i passi lontani, il che e' corretto.

    Ritorna anche gli errori del PRIMO passo (err_fut/err_persist) e i veri del
    primo passo (true_real) per il pooling tra trial, che resta one-step.
    """
    pred_norm = np.asarray(pred_norm)
    true_norm = np.asarray(true_norm)
    if pred_norm.ndim == 1:
        pred_norm = pred_norm[:, None]
        true_norm = true_norm[:, None]
    n, P = pred_norm.shape

    # --- per-passo: RMSE/MAE del passo k in unita' reali ---
    rmse_k, mae_k = [], []
    rmse_persist_k, mae_persist_k = [], []
    skill_rmse_k, skill_mae_k = [], []
    rmse_pct_k, mae_pct_k = [], []
    persist_pct_k = []
    # errori normalizzati per-passo, da accumulare tra trial per il pooled
    # per-orizzonte: err_by_step[k] = errori reali del passo k in questo trial.
    err_fut_by_step, err_persist_by_step, true_by_step = [], [], []
    for k in range(P):
        e_f = _real_err(pred_norm[:, k], true_norm[:, k], scaler)
        r_f, m_f = rmse_mae(e_f)
        # persistenza per il passo k: predici sempre 'last' (vero a t)
        e_p = _real_err(last_norm, true_norm[:, k], scaler)
        r_p, m_p = rmse_mae(e_p)
        true_real_k = scaler.inverse_transform(
            np.asarray(true_norm[:, k]).reshape(-1, 1)).ravel()
        ppk_k = float(true_real_k.max() - true_real_k.min())

        rmse_k.append(r_f); mae_k.append(m_f)
        rmse_persist_k.append(r_p); mae_persist_k.append(m_p)
        rmse_pct_k.append(100.0 * r_f / ppk_k if ppk_k > 0 else float("nan"))
        mae_pct_k.append(100.0 * m_f / ppk_k if ppk_k > 0 else float("nan"))
        persist_pct_k.append(100.0 * r_p / ppk_k if ppk_k > 0 else float("nan"))
        skill_rmse_k.append((1.0 - r_f / r_p) if r_p > 0 else float("nan"))
        skill_mae_k.append((1.0 - m_f / m_p) if m_p > 0 else float("nan"))

        err_fut_by_step.append(e_f)
        err_persist_by_step.append(e_p)
        true_by_step.append(true_real_k)

    def _avg(a):
        a = np.asarray(a, dtype=float)
        return float(np.nanmean(a)) if a.size else float("nan")

    # --- primo passo (t+1): per la tabella e per il pooling tra trial ---
    e_f_1 = _real_err(pred_norm[:, 0], true_norm[:, 0], scaler)
    e_p_1 = _real_err(last_norm,       true_norm[:, 0], scaler)
    true_real_1 = scaler.inverse_transform(
        np.asarray(true_norm[:, 0]).reshape(-1, 1)).ravel()

    return {
        "P": P,
        # primo passo
        "rmse_f": rmse_k[0], "mae_f": mae_k[0],
        "rmse_persist": rmse_persist_k[0], "mae_persist": mae_persist_k[0],
        "rmse_fut_pct": rmse_pct_k[0], "persist_pct": persist_pct_k[0],
        "mae_fut_pct": mae_pct_k[0],
        "skill_rmse": skill_rmse_k[0], "skill_mae": skill_mae_k[0],
        # media per-passo sull'orizzonte
        "rmse_f_avg": _avg(rmse_k), "mae_f_avg": _avg(mae_k),
        "rmse_fut_pct_avg": _avg(rmse_pct_k), "mae_fut_pct_avg": _avg(mae_pct_k),
        "skill_rmse_avg": _avg(skill_rmse_k), "skill_mae_avg": _avg(skill_mae_k),
        # pooling one-step tra trial (primo passo)
        "err_fut": e_f_1, "err_persist": e_p_1, "true_real": true_real_1,
        # pooling per-orizzonte tra trial (liste lunghe P)
        "err_fut_by_step": err_fut_by_step,
        "err_persist_by_step": err_persist_by_step,
        "true_by_step": true_by_step,
    }


def build_wide_row(trial_name, split, n_win, per_channel, channels):
    row = {"trial": trial_name, "split": split, "n_finestre": n_win}
    for ch in channels:
        m = per_channel[ch]
        row[f"{ch}_net"]              = CH_NET[ch]
        row[f"{ch}_P"]               = m["P"]
        # --- primo passo (t+1) ---
        row[f"{ch}_RMSE_1"]          = m["rmse_f"]
        row[f"{ch}_RMSE_pct_1"]      = m["rmse_fut_pct"]
        row[f"{ch}_skill_rmse_1"]    = m["skill_rmse"]
        row[f"{ch}_MAE_1"]           = m["mae_f"]
        row[f"{ch}_MAE_pct_1"]       = m["mae_fut_pct"]
        row[f"{ch}_skill_mae_1"]     = m["skill_mae"]
        # --- media sui P passi ---
        row[f"{ch}_RMSE_avg"]        = m["rmse_f_avg"]
        row[f"{ch}_RMSE_pct_avg"]    = m["rmse_fut_pct_avg"]
        row[f"{ch}_skill_rmse_avg"]  = m["skill_rmse_avg"]
        row[f"{ch}_MAE_avg"]         = m["mae_f_avg"]
        row[f"{ch}_MAE_pct_avg"]     = m["mae_fut_pct_avg"]
        row[f"{ch}_skill_mae_avg"]   = m["skill_mae_avg"]
        # baseline persistenza (primo passo), utile come riferimento
        row[f"{ch}_RMSE_persist"]    = m["rmse_persist"]
        row[f"{ch}_MAE_persist"]     = m["mae_persist"]
    return row


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--checkpoint", default=os.path.join(SCRIPT_DIR, "checkpoints_joint", "best.pt"))
    ap.add_argument("--dataset_dir", default=os.path.join(REPO_ROOT, "src", "net", "dataset"))
    ap.add_argument("--scaler_path", default=os.path.join(REPO_ROOT, "src", "net", "scaler", "scalers_joint.pkl"),
                    help="normalizzatore condiviso: DEVE essere lo stesso del training")
    ap.add_argument("--device", default="cuda" if torch.cuda.is_available() else "cpu")
    ap.add_argument("--all_trials", action="store_true",
                    help="includi anche i trial di train (default: solo validation)")
    ap.add_argument("--csv_out", default=os.path.join(SCRIPT_DIR, "metrics_per_trial_joint.csv"))
    ap.add_argument("--p", type=int, default=None,
                    help="orizzonte P per costruire i target del dataset. Default: "
                         "il P salvato nel checkpoint (consigliato). Passalo solo "
                         "per forzare un valore diverso.")
    args = ap.parse_args()

    device = torch.device(args.device)

    # carico prima il checkpoint per conoscere il P con cui e' stato allenato:
    # il dataset DEVE costruire target con lo stesso orizzonte, altrimenti si
    # valuta un P diverso da quello del modello (bug silenzioso).
    ckpt = torch.load(args.checkpoint, map_location=device, weights_only=False)
    P_ckpt = ckpt.get("P", 1)
    P = args.p if args.p is not None else P_ckpt
    if args.p is not None and args.p != P_ckpt:
        print(f"[avviso] --p={args.p} diverso dal P del checkpoint ({P_ckpt}): "
              f"uso --p={args.p}. Le metriche potrebbero non corrispondere al "
              f"modello allenato.")
    print(f"P usato per il dataset: {P} (checkpoint: {P_ckpt})")

    # stessa normalizzazione e stesso split del training, con l'orizzonte P giusto
    dataset = FishJointDataset(args.dataset_dir, p=P, scaler_path=args.scaler_path)
    _, val_ds = dataset.split_by_trial(val_frac=VAL_FRAC, seed=SPLIT_SEED)
    val_trials = set(int(i) for i in np.unique(
        dataset.window_trial[np.asarray(val_ds.indices)]))

    ctx_dim = ckpt.get("ctx_dim", int(dataset.context.shape[-1]))
    gh_im, mh_im = dims_from_state(ckpt["im_state"])
    gh_fm, mh_fm = dims_from_state(ckpt["fm_state"])
    print(f"Dimensioni dal checkpoint: IM gru={gh_im} mlp={mh_im} | "
          f"FM gru={gh_fm} mlp={mh_fm} | P={P} | ctx_dim={ctx_dim}")

    # coerenza contesto
    ds_ctx = int(dataset.context.shape[-1])
    if ctx_dim != ds_ctx:
        raise ValueError(
            f"MISMATCH contesto: checkpoint ctx_dim={ctx_dim} ma il dataset ne "
            f"produce {ds_ctx}. Controlla CTX_DIM e la costruzione del contesto.")

    IM, FM = build_models(gru_hidden_im=gh_im, mlp_hidden_im=mh_im,
                          gru_hidden_fm=gh_fm, mlp_hidden_fm=mh_fm,
                          p=P, ctx_dim=ctx_dim)
    IM.load_state_dict(ckpt["im_state"]); IM.to(device).eval()
    FM.load_state_dict(ckpt["fm_state"]); FM.to(device).eval()

    # coerenza canali di uscita
    if FM.out_channels != len(FM_CHANNELS):
        raise ValueError(f"FM predice {FM.out_channels} canali, attesi {len(FM_CHANNELS)}.")
    if IM.out_channels != len(IM_CHANNELS):
        raise ValueError(f"IM predice {IM.out_channels} canali, attesi {len(IM_CHANNELS)}.")

    ALL_CHANNELS = FM_CHANNELS + IM_CHANNELS   # [sensor_diff, current, cmd]
    print(f"canali: FM={FM_CHANNELS} | IM={IM_CHANNELS}")

    if args.all_trials:
        trials = list(range(len(dataset.trial_names)))
    else:
        trials = sorted(val_trials)
    print(f"trial considerati: {len(trials)} "
          f"({'tutti' if args.all_trials else 'solo validation'})")

    # accumulatori pooled PER-PASSO: per ogni canale e passo k, gli errori reali
    # di tutte le finestre di tutti i trial. Cosi' il pooled e' per-orizzonte.
    err_pool = {ch: {"fut": None, "persist": None, "true": None}
                for ch in ALL_CHANNELS}

    rows = []
    metric_dicts = []   # i dict per-canale di ogni trial, per la media-per-trial
    for ti in trials:
        out = run_trial(dataset, IM, FM, device, ti)
        if out is None:
            continue
        split = "val" if ti in val_trials else "train"
        n_win = len(out["tgt"]["cmd"])

        per_channel = {}
        for ch in ALL_CHANNELS:
            sc = dataset.scalers[CH_TO_KEY[ch]]
            pred, true, last = channel_arrays(out, ch)
            m = channel_metrics(pred, true, last, sc)
            per_channel[ch] = m

            # accumulo per-passo (liste lunghe P): inizializzo alla prima volta
            P_ch = m["P"]
            if err_pool[ch]["fut"] is None:
                err_pool[ch]["fut"]     = [[] for _ in range(P_ch)]
                err_pool[ch]["persist"] = [[] for _ in range(P_ch)]
                err_pool[ch]["true"]    = [[] for _ in range(P_ch)]
            for k in range(P_ch):
                err_pool[ch]["fut"][k].append(m["err_fut_by_step"][k])
                err_pool[ch]["persist"][k].append(m["err_persist_by_step"][k])
                err_pool[ch]["true"][k].append(m["true_by_step"][k])

        metric_dicts.append(per_channel)
        rows.append(build_wide_row(dataset.trial_names[ti], split, n_win,
                                   per_channel, ALL_CHANNELS))

    split_rank = {"val": 0, "train": 1}
    rows.sort(key=lambda r: (split_rank.get(r["split"], 2), r["trial"]))

    P_seen = rows[0][f"{ALL_CHANNELS[0]}_P"] if rows else 1

    # --- MEDIA PER-TRIAL (ogni trial pesa uguale) ---
    # media aritmetica delle metriche dei singoli trial. Per gli skill (rapporti)
    # medio gli skill dei trial, non li ricalcolo dai RMSE medi.
    def _nanmean(vals):
        a = np.asarray(vals, dtype=float)
        return float(np.nanmean(a)) if a.size else float("nan")

    mean_channel = {}
    for ch in ALL_CHANNELS:
        keys = ["rmse_f", "mae_f", "rmse_persist", "mae_persist",
                "rmse_fut_pct", "mae_fut_pct", "skill_rmse", "skill_mae",
                "rmse_f_avg", "mae_f_avg", "rmse_fut_pct_avg", "mae_fut_pct_avg",
                "skill_rmse_avg", "skill_mae_avg"]
        agg = {k: _nanmean([md[ch][k] for md in metric_dicts]) for k in keys}
        agg["P"] = P_seen
        mean_channel[ch] = agg
    mean_split = "MEDIA_val" if not args.all_trials else "MEDIA_all"
    n_trials_used = len(metric_dicts)
    mean_row = build_wide_row("== MEDIA per-trial ==", mean_split,
                              n_trials_used, mean_channel, ALL_CHANNELS)

    # --- POOLED PER-ORIZZONTE (tutte le finestre insieme, pesate per lunghezza) ---
    # Per ogni passo k: RMSE/MAE pooled su tutti i trial. Colonna '_1' = passo 0,
    # '_avg' = media dei pooled sui P passi.
    pooled_channel = {}
    n_pool = 0
    for ch in ALL_CHANNELS:
        Pn = len(err_pool[ch]["fut"])
        rmse_k, mae_k = [], []
        rmse_p_k, mae_p_k = [], []
        rmse_pct_k, mae_pct_k = [], []
        skill_rmse_k, skill_mae_k = [], []
        n_k0 = 0
        for k in range(Pn):
            e_f  = np.concatenate(err_pool[ch]["fut"][k])
            e_p  = np.concatenate(err_pool[ch]["persist"][k])
            true = np.concatenate(err_pool[ch]["true"][k])
            r_f, m_f = rmse_mae(e_f)
            r_p, m_p = rmse_mae(e_p)
            ppk = float(true.max() - true.min())
            rmse_k.append(r_f); mae_k.append(m_f)
            rmse_p_k.append(r_p); mae_p_k.append(m_p)
            rmse_pct_k.append(100.0 * r_f / ppk if ppk > 0 else float("nan"))
            mae_pct_k.append(100.0 * m_f / ppk if ppk > 0 else float("nan"))
            skill_rmse_k.append((1.0 - r_f / r_p) if r_p > 0 else float("nan"))
            skill_mae_k.append((1.0 - m_f / m_p) if m_p > 0 else float("nan"))
            if k == 0:
                n_k0 = len(e_f)

        def _avg(a):
            a = np.asarray(a, dtype=float)
            return float(np.nanmean(a)) if a.size else float("nan")

        pooled_channel[ch] = {
            "P": P_seen,
            # primo passo (t+1) pooled
            "rmse_f": rmse_k[0], "mae_f": mae_k[0],
            "rmse_persist": rmse_p_k[0], "mae_persist": mae_p_k[0],
            "rmse_fut_pct": rmse_pct_k[0], "mae_fut_pct": mae_pct_k[0],
            "skill_rmse": skill_rmse_k[0], "skill_mae": skill_mae_k[0],
            # media sui P passi dei pooled per-passo
            "rmse_f_avg": _avg(rmse_k), "mae_f_avg": _avg(mae_k),
            "rmse_fut_pct_avg": _avg(rmse_pct_k), "mae_fut_pct_avg": _avg(mae_pct_k),
            "skill_rmse_avg": _avg(skill_rmse_k), "skill_mae_avg": _avg(skill_mae_k),
        }
        n_pool = n_k0
    pooled_split = "POOLED_val" if not args.all_trials else "POOLED_all"
    pooled_row = build_wide_row("== COMPLESSIVO (pooled) ==", pooled_split,
                                n_pool, pooled_channel, ALL_CHANNELS)

    # media-per-trial e pooled in coda, in quest'ordine
    rows.append(mean_row)
    rows.append(pooled_row)

    # ordine colonne: per ogni canale, blocco 'primo passo' e blocco 'media P'
    lead = ["trial", "split", "n_finestre"]
    metric_cols = []
    for ch in ALL_CHANNELS:
        metric_cols += [
            f"{ch}_net", f"{ch}_P",
            f"{ch}_RMSE_1", f"{ch}_RMSE_pct_1", f"{ch}_skill_rmse_1",
            f"{ch}_MAE_1",  f"{ch}_MAE_pct_1",  f"{ch}_skill_mae_1",
            f"{ch}_RMSE_avg", f"{ch}_RMSE_pct_avg", f"{ch}_skill_rmse_avg",
            f"{ch}_MAE_avg",  f"{ch}_MAE_pct_avg",  f"{ch}_skill_mae_avg",
            f"{ch}_RMSE_persist", f"{ch}_MAE_persist",
        ]
    df_out = pd.DataFrame(rows)[lead + metric_cols]

    # arrotonda solo le colonne float (i _net sono stringhe, i _P sono interi)
    num_cols = [c for c in metric_cols
                if not c.endswith("_net") and not c.endswith("_P")]
    df_out[num_cols] = df_out[num_cols].round(ROUND_DEC)

    pd.set_option("display.width", 260)
    pd.set_option("display.max_columns", None)
    print()
    print(df_out.to_string(index=False))

    df_out.to_csv(args.csv_out, index=False)
    print(f"\nTabella salvata in {args.csv_out}")
    print("Unita': " + ", ".join(f"{ch}={CH_UNIT[ch]}" for ch in ALL_CHANNELS))

    # --- verdetto sintetico sul POOLED (primo passo t+1) ---
    print(f"\n=== Verdetto (pooled per-orizzonte, primo passo t+1 | P={P_seen}) ===")
    for ch in ALL_CHANNELS:
        m = pooled_channel[ch]
        verdict = ("batte la persistenza" if m["skill_rmse"] > 0.05 else
                   "~come la persistenza" if m["skill_rmse"] > -0.05 else
                   "PEGGIO della persistenza")
        line = (f"  [{CH_NET[ch]}] {ch:<12} RMSE {m['rmse_f']:.3f} "
                f"({m['rmse_fut_pct']:.1f}% ppk) | MAE {m['mae_f']:.3f} "
                f"| persist RMSE {m['rmse_persist']:.3f} "
                f"| skill_rmse {m['skill_rmse']:+.2f} -> {verdict}")
        if P_seen > 1:
            line += f"  |  avg su P: RMSE {m['rmse_f_avg']:.3f} skill {m['skill_rmse_avg']:+.2f}"
        print(line)
    print("Righe riassuntive nella tabella: '== MEDIA per-trial ==' (ogni trial "
          "pesa uguale) e '== COMPLESSIVO (pooled) ==' (pesata per finestre).")
    if P_seen > 1:
        print("Colonne *_1 = passo t+1 ; *_avg = media sui P passi dell'orizzonte.")


if __name__ == "__main__":
    main()