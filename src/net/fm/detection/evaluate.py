"""
VALUTAZIONE SISTEMATICA del detector con perturbazioni sintetiche iniettate.

Per ogni trial (default: i trial di validazione), ogni canale, ogni scenario e
ogni ampiezza fa un replay completo nel detector e misura:
  detector veloce   rilevato?  ritardo (allarme - inizio perturbazione)
                    classe giusta (transitoria / persistente)
                    inizio giusto (brusco per gli step, graduale per le rampe)
                    canale giusto (il canale perturbato e' classificato)
                    solo il canale giusto (l'altro e' "nessuna")
  detector lento    rilevato sul canale perturbato?  ritardo
                    origine giusta (il primo allarme lento e' sul canale perturbato)
  falsi allarmi     allarmi prima dell'inizio della perturbazione e sui trial puliti

Scenari (ampiezze in deviazioni standard del segnale, spazio normalizzato):
  step_1s    step di 1 s                     atteso: transitoria, brusco
  step_3s    step di 3 s                     atteso: transitoria, brusco
  step_pers  step fino a fine trial          atteso: persistente, brusco
  ramp_pers  rampa di 5 s, poi costante      atteso: persistente, graduale
  gain_pers  segnale * (1 + a) fino a fine   atteso: persistente (inizio non valutato)
L'istante d'inizio e' casuale in [t0_min, t0_max], uguale per tutte le ampiezze
dello stesso (trial, scenario, canale, ripetizione): le curve per ampiezza sono
quindi confrontabili.

ATTENZIONE
- Le soglie sono calibrate sugli stessi trial di validazione: i falsi allarmi
  sui trial puliti sono zero per costruzione. La stima onesta dei falsi
  allarmi e' il leave-one-trial-out di calibrate.py, che qui viene ristampato.
- Una perturbazione additiva sul sensore e' un surrogato di quelle vere.

Uso (dalla cartella aquabot):
  python3 src/net/fm/detection/evaluate.py --checkpoint src/net/fm/checkpoints_fm/best_fm_tuned.pt
  ... --workers 4 --reps 2
  ... --amps_step 0.5,1,2 --amps_ramp 1,2 --amps_gain 0.25,0.5    (prova veloce)

Uscite in --out (default detection_out/evaluate):
  evaluate_runs.csv      una riga per replay
  evaluate_summary.csv   riepilogo per canale / scenario / ampiezza
  evaluate_summary.png   tasso di rilevamento e ritardo in funzione dell'ampiezza
"""
import argparse
import multiprocessing as mp
import os
import time
import zlib

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import torch

from det_common import (DEFAULT_DATASET, DEFAULT_SCALER, SCRIPT_DIR, default_calib_path,
                        load_dataset, load_model, replay, trial_series)
from detector import CHANNELS, M1, MR, Calibration, PerturbationDetector
from run_detector import apply_injections

COL_CH = {"sensor_diff": "#2a78d6", "current": "#d9822b"}

# nome: (tipo, durata in s (0 = fino a fine trial), classe attesa, inizio atteso)
SCENARIOS = {
    "step_1s":   ("step", 1.0, "transitoria", "brusco"),
    "step_3s":   ("step", 3.0, "transitoria", "brusco"),
    "step_pers": ("step", 0.0, "persistente", "brusco"),
    "ramp_pers": ("ramp", 5.0, "persistente", "graduale"),
    "gain_pers": ("gain", 0.0, "persistente", None),
}

# variabili globali dei processi di lavoro (ereditate con fork)
G = {}


def floats(s):
    return [float(x) for x in s.split(",") if x.strip()]


def job_t0(seed, trial, rep, scen, ch, lo, hi):
    """Istante d'inizio riproducibile, uguale per tutte le ampiezze."""
    key = zlib.crc32(f"{seed}|{trial}|{rep}|{scen}|{ch}".encode())
    return float(np.round(np.random.default_rng(key).uniform(lo, hi), 1))


def run_job(job):
    ti, name, scen, ch, amp, t0, rep = job
    calib, hz = G["calib"], G["calib"].config.hz
    cmd, sens = G["series"][ti]
    n = len(cmd)
    det = PerturbationDetector(G["FM"], calib, H=G["H"], device="cpu",
                               norm_stats=G["norm_stats"])

    if scen == "pulito":
        r = replay(det, cmd, sens)
        ratio = r["score"][:, MR, :] / calib.threshold[MR]
        gh = r["g"] / calib.cusum_h
        return {"trial": name, "scenario": scen, "canale": "-", "amp": 0.0, "rep": rep,
                "t0": np.nan,
                "n_allarmi": int(r["alarm"].sum()), "n_allarmi_lenti": len(det.slow_events),
                "max_SR_sd": float(np.nanmax(ratio[:, 0])), "max_SR_cur": float(np.nanmax(ratio[:, 1])),
                "max_g_sd": float(np.nanmax(gh[:, 0])), "max_g_cur": float(np.nanmax(gh[:, 1]))}

    kind, dur, exp_cls, exp_onset = SCENARIOS[scen]
    sens_p, _ = apply_injections(sens, [(ch, kind, t0, dur, amp)], hz)
    r = replay(det, cmd, sens_p)

    ci = CHANNELS.index(ch)
    other = CHANNELS[1 - ci]
    i0 = int(round(t0 * hz))
    persistent = exp_cls == "persistente"
    pert_end = n if (persistent or dur <= 0) else int(round((t0 + dur) * hz))
    # finestra in cui un allarme conta come rilevamento: per le transitorie fino
    # a 1 s + W dopo la fine della perturbazione
    win_end = n if persistent else min(n, pert_end + det.nW + int(hz))

    alarms = np.flatnonzero(r["alarm"])
    hit = alarms[(alarms >= i0) & (alarms < win_end)]
    row = {"trial": name, "scenario": scen, "canale": ch, "amp": amp, "rep": rep, "t0": t0,
           "fa_prima": bool((alarms < i0).any()),
           "fa_dopo": int((alarms >= win_end).sum()),
           "n_allarmi_finestra": int(len(hit)),
           "rilevato": bool(len(hit)), "ritardo_s": np.nan,
           "classificato": False, "classe": None, "classe_ok": np.nan,
           "inizio": None, "inizio_ok": np.nan, "trigger": None,
           "canale_ok": np.nan, "solo_canale": np.nan}
    if len(hit):
        a = int(hit[0])
        row["ritardo_s"] = (a - i0) / hz
        ev = next((e for e in det.events if e["t_alarm"] == a), None)
        if ev is not None:
            chans = ev["channels"]
            row.update(classificato=True, classe=ev["classe"],
                       classe_ok=float(ev["classe"] == exp_cls),
                       trigger="+".join(ev["trigger"]),
                       inizio=chans[ch]["inizio"],
                       persistente_da=chans[ch].get("persistente_da"),
                       canale_ok=float(chans[ch]["classe"] != "nessuna"),
                       solo_canale=float(chans[ch]["classe"] != "nessuna"
                                         and chans[other]["classe"] == "nessuna"))
            if exp_onset is not None and chans[ch]["inizio"] is not None:
                row["inizio_ok"] = float(chans[ch]["inizio"] == exp_onset)

    # ---- detector lento ----
    slow = sorted(det.slow_events, key=lambda e: (e["t_alarm"], e["secondario"]))
    row["fa_lento_prima"] = any(e["t_alarm"] < i0 for e in slow)
    after = [e for e in slow if e["t_alarm"] >= i0]
    mine = [e for e in after if e["canale"] == ch]
    row["lento_rilevato"] = bool(mine)
    row["lento_ritardo_s"] = (mine[0]["t_alarm"] - i0) / hz if mine else np.nan
    row["lento_origine_ok"] = (float(after[0]["canale"] == ch and not after[0]["secondario"])
                               if after else np.nan)
    row["lento_altro_canale"] = any(e["canale"] == other and not e["secondario"] for e in after)
    return row


def pct(s):
    s = pd.to_numeric(s, errors="coerce").dropna()
    return 100.0 * s.mean() if len(s) else np.nan


def summarize(df):
    rows = []
    for (ch, scen, amp), d in df.groupby(["canale", "scenario", "amp"], sort=False):
        det = d[d.rilevato]
        rows.append({
            "canale": ch, "scenario": scen, "amp": amp, "n": len(d),
            "rilev_%": pct(d.rilevato),
            "ritardo_med_s": det.ritardo_s.median(),
            "classe_ok_%": pct(det.classe_ok),
            "inizio_ok_%": pct(det.inizio_ok),
            "canale_ok_%": pct(det.canale_ok),
            "solo_canale_%": pct(det.solo_canale),
            "allarmi_extra": (d.n_allarmi_finestra - 1).clip(lower=0).mean() + d.fa_dopo.mean(),
            "lento_rilev_%": pct(d.lento_rilevato),
            "lento_ritardo_med_s": d.lento_ritardo_s.median(),
            "lento_origine_ok_%": pct(d.lento_origine_ok),
            "fa_prima": int(d.fa_prima.sum() + d.fa_lento_prima.sum()),
        })
    return pd.DataFrame(rows)


def min_amp(summary, col, target):
    """Ampiezza minima da cui in poi il tasso resta >= target (NaN se mai)."""
    out = {}
    for (ch, scen), d in summary.groupby(["canale", "scenario"], sort=False):
        d = d.sort_values("amp")
        ok = (d[col] >= target).to_numpy()
        tail = np.flatnonzero(~ok)
        if ok.all():
            out[(ch, scen)] = d.amp.iloc[0]
        elif tail[-1] + 1 < len(d):
            out[(ch, scen)] = d.amp.iloc[tail[-1] + 1]
        else:
            out[(ch, scen)] = np.nan
    return out


def plot(summary, path):
    scen = list(dict.fromkeys(summary.scenario))
    fig, axes = plt.subplots(2, len(scen), figsize=(3.6 * len(scen), 6.2), squeeze=False)
    for j, s in enumerate(scen):
        d = summary[summary.scenario == s]
        for ch in CHANNELS:
            e = d[d.canale == ch].sort_values("amp")
            if e.empty:
                continue
            c = COL_CH[ch]
            axes[0, j].plot(e.amp, e["rilev_%"], "o-", color=c, label=f"{ch} veloce")
            axes[0, j].plot(e.amp, e["lento_rilev_%"], "s--", color=c, alpha=0.7, label=f"{ch} lento")
            axes[1, j].plot(e.amp, e["ritardo_med_s"], "o-", color=c)
            axes[1, j].plot(e.amp, e["lento_ritardo_med_s"], "s--", color=c, alpha=0.7)
        axes[0, j].set_title(s, fontsize=10)
        axes[0, j].set_ylim(-5, 105)
        axes[1, j].set_xlabel("ampiezza (dev. std del segnale)" if s != "gain_pers" else "guadagno a")
        for ax in axes[:, j]:
            ax.grid(True, alpha=0.3)
    axes[0, 0].set_ylabel("rilevati (%)")
    axes[1, 0].set_ylabel("ritardo mediano (s)")
    axes[0, 0].legend(fontsize=7)
    fig.suptitle("Valutazione con perturbazioni iniettate - linea piena: veloce, tratteggiata: lento",
                 fontsize=10)
    fig.tight_layout()
    fig.savefig(path, dpi=130)
    plt.close(fig)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--checkpoint", required=True)
    ap.add_argument("--calib", default=None)
    ap.add_argument("--dataset_dir", default=DEFAULT_DATASET)
    ap.add_argument("--scaler_path", default=DEFAULT_SCALER)
    ap.add_argument("--trials", default="val", help="'val' (default), 'all' o nomi separati da virgola")
    ap.add_argument("--channels", default="sensor_diff,current")
    ap.add_argument("--scenarios", default=",".join(SCENARIOS))
    ap.add_argument("--amps_step", default="0.25,0.5,1,1.5,2,3")
    ap.add_argument("--amps_ramp", default="0.25,0.5,1,2")
    ap.add_argument("--amps_gain", default="0.1,0.25,0.5,1")
    ap.add_argument("--reps", type=int, default=1, help="istanti d'inizio casuali per trial")
    ap.add_argument("--t0_min", type=float, default=5.0)
    ap.add_argument("--t0_max", type=float, default=15.0)
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--workers", type=int, default=max(1, (os.cpu_count() or 2) - 1))
    ap.add_argument("--no_persist_cusum", action="store_true",
                    help="classificazione persistente solo da S_R (regola vecchia, per confronto)")
    ap.add_argument("--out", default=os.path.join(SCRIPT_DIR, "detection_out", "evaluate"))
    args = ap.parse_args()

    FM, ckpt = load_model(args.checkpoint, torch.device("cpu"))
    calib = Calibration.load(args.calib or default_calib_path(args.checkpoint))
    if args.no_persist_cusum:
        calib.config.persist_cusum = False
    print(f"persist_cusum = {calib.config.persist_cusum}")
    dataset, val_trials = load_dataset(args.dataset_dir, args.scaler_path, ckpt)
    names = dataset.trial_names
    if args.trials == "val":
        tids = val_trials
    elif args.trials == "all":
        tids = list(range(len(names)))
    else:
        tids = [names.index(x.strip()) for x in args.trials.split(",")]
    hz = calib.config.hz
    G.update(FM=FM, calib=calib, H=dataset.h, norm_stats=ckpt.get("norm_stats"),
             series={ti: trial_series(dataset, ti) for ti in tids})

    short = min(len(G["series"][ti][0]) for ti in tids) / hz
    if args.t0_max + calib.config.T_s + 3 > short:
        print(f"[attenzione] trial lunghi {short:.1f} s: con t0_max={args.t0_max} alcune "
              f"perturbazioni potrebbero non essere classificate prima della fine.")

    amps = {"step": floats(args.amps_step), "ramp": floats(args.amps_ramp), "gain": floats(args.amps_gain)}
    chans = [c.strip() for c in args.channels.split(",")]
    scens = [s.strip() for s in args.scenarios.split(",")]
    for s in scens:
        if s not in SCENARIOS:
            raise SystemExit(f"scenario sconosciuto: {s} (validi: {list(SCENARIOS)})")

    jobs = [(ti, names[ti], "pulito", "-", 0.0, np.nan, 0) for ti in tids]
    for ti in tids:
        for rep in range(args.reps):
            for s in scens:
                for ch in chans:
                    t0 = job_t0(args.seed, names[ti], rep, s, ch, args.t0_min, args.t0_max)
                    for a in amps[SCENARIOS[s][0]]:
                        jobs.append((ti, names[ti], s, ch, a, t0, rep))

    print(f"{len(tids)} trial, {len(jobs)} replay, {args.workers} processi")
    ns = ckpt.get("norm_stats")
    if ns:
        print(f"Ampiezza 1 = {ns['sd_std']:.1f} sensor units su sensor_diff, "
              f"{ns['vf_std']:.1f} mA su current")
    print(f"Ampiezza 1 = {1 / calib.sigma[M1, 0]:.1f} sigma del residuo one-step su sensor_diff, "
          f"{1 / calib.sigma[M1, 1]:.1f} su current")

    t_start = time.time()
    rows = []
    if args.workers > 1:
        ctx = mp.get_context("fork")
        with ctx.Pool(args.workers) as pool:
            for k, row in enumerate(pool.imap_unordered(run_job, jobs, chunksize=4), 1):
                rows.append(row)
                el = time.time() - t_start
                print(f"\r  {k}/{len(jobs)}  trascorsi {el / 60:.1f} min, "
                      f"mancano ~{el / k * (len(jobs) - k) / 60:.1f} min   ", end="", flush=True)
    else:
        for k, job in enumerate(jobs, 1):
            rows.append(run_job(job))
            el = time.time() - t_start
            print(f"\r  {k}/{len(jobs)}  trascorsi {el / 60:.1f} min, "
                  f"mancano ~{el / k * (len(jobs) - k) / 60:.1f} min   ", end="", flush=True)
    print()

    os.makedirs(args.out, exist_ok=True)
    df = pd.DataFrame(rows)
    clean = df[df.scenario == "pulito"]
    pert = df[df.scenario != "pulito"].copy()
    order = {s: i for i, s in enumerate(scens)}
    pert["_o"] = pert.scenario.map(order)
    pert = pert.sort_values(["canale", "_o", "amp", "trial", "rep"]).drop(columns="_o")
    df_out = pd.concat([pert, clean], ignore_index=True)
    df_out.to_csv(os.path.join(args.out, "evaluate_runs.csv"), index=False)

    # ---- trial puliti ----
    print("\n=== Trial puliti (soglie calibrate su questi stessi trial) ===")
    print(f"  allarmi veloci: {int(clean.n_allarmi.sum())} | allarmi lenti: "
          f"{int(clean.n_allarmi_lenti.sum())} su {len(clean)} trial")
    print(f"  max S_R/soglia: sensor_diff {clean.max_SR_sd.max():.2f}, current {clean.max_SR_cur.max():.2f}"
          f" | max g/h: sensor_diff {clean.max_g_sd.max():.2f}, current {clean.max_g_cur.max():.2f}")
    meta = calib.meta
    if "loo_false_alarm_rate_per_trial" in meta:
        print(f"  leave-one-trial-out (da calibrate.py): falsi allarmi veloci in "
              f"{100 * meta['loo_false_alarm_rate_per_trial']:.0f}% dei trial")
    for v, cu in meta.get("cusum", {}).items():
        if "loo_max" in cu:
            lm = cu["loo_max"]
            print(f"  leave-one-trial-out CUSUM ({v}): max g/h sensor_diff {lm['sensor_diff']:.2f}, "
                  f"current {lm['current']:.2f} (>1 = falso allarme lento)")

    # ---- riepilogo ----
    summary = summarize(pert)
    summary.to_csv(os.path.join(args.out, "evaluate_summary.csv"), index=False)
    pd.set_option("display.width", 250)
    pd.set_option("display.max_columns", None)
    print("\n=== Perturbazioni iniettate ===")
    print("rilev_%       allarme veloce durante la perturbazione (transitorie: fino a 1 s + W dopo)")
    print("classe/inizio/canale_ok_%   tra quelli rilevati | solo_canale_% = l'altro canale e' 'nessuna'")
    print("allarmi_extra  allarmi veloci in piu' per replay (frammentazione o dopo la fine)")
    print("lento_*       detector lento sul canale perturbato | origine_ok = primo allarme lento sul canale giusto")
    print("fa_prima      allarmi (veloci + lenti) prima dell'inizio della perturbazione\n")
    for ch in chans:
        print(f"--- {ch} ---")
        print(summary[summary.canale == ch].drop(columns="canale").round(2).to_string(index=False))
        print()

    for col, lab in [("rilev_%", "veloce"), ("lento_rilev_%", "lento")]:
        m = min_amp(summary, col, 90.0)
        print(f"Ampiezza minima con rilevamento {lab} >= 90%:")
        for (ch, s), a in m.items():
            print(f"  {ch:12s} {s:10s} {'mai' if np.isnan(a) else f'{a:g}'}")
    plot(summary, os.path.join(args.out, "evaluate_summary.png"))
    print(f"\nSalvati in {args.out}/: evaluate_runs.csv, evaluate_summary.csv, evaluate_summary.png")


if __name__ == "__main__":
    main()