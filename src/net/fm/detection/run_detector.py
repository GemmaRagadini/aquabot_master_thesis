"""
REPLAY di un trial nel detector online, campione per campione, con figura e
riepilogo degli eventi. Opzionalmente inietta perturbazioni SINTETICHE sui
sensori per provare la catena allarme -> classificazione.

--inject canale:tipo:t_inizio:durata:ampiezza   (ripetibile)
  canale    sensor_diff | current
  tipo      step  offset costante per `durata` secondi (durata 0 = fino alla fine)
            ramp  cresce linearmente fino ad `ampiezza` in `durata` s, poi resta
            gain  moltiplica il segnale per (1 + ampiezza) per `durata` secondi
  ampiezza  in deviazioni standard del segnale (spazio normalizzato)
  es: --inject sensor_diff:step:10:1:2      transitoria brusca di 1 s
      --inject current:ramp:8:6:1.5         persistente graduale

ATTENZIONE: una perturbazione additiva sul sensore e' un surrogato. Quelle
vere cambiano la dinamica; servono a verificare la logica, non la sensibilita'.

Uso:
  python3 src/net/fm/detection/run_detector.py --checkpoint .../best_fm_tuned.pt
  python3 .../run_detector.py --checkpoint ... --trial trial_106.csv --inject sensor_diff:step:10:1:2
"""
import argparse
import json
import os
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import torch

from det_common import (CH_TO_KEY, CH_UNIT, DEFAULT_DATASET, DEFAULT_SCALER, SCRIPT_DIR,
                        default_calib_path, load_dataset, load_model, replay,
                        resolve_trial, trial_series)
from detector import (ALLARME, CHANNELS, M1, MR, PERSISTENTE, Calibration,
                      PerturbationDetector)

COL_SURFACE, COL_TEXT, COL_TEXT_SEC = "#fcfcfb", "#0b0b0b", "#52514e"
COL_MUTED, COL_GRID, COL_BASELINE   = "#898781", "#e1e0d9", "#c3c2b7"
COL_SIGNAL, COL_ONE, COL_ROLL       = "#0b0b0b", "#2a78d6", "#d9822b"
COL_CH        = {"sensor_diff": "#2a78d6", "current": "#d9822b"}
COL_STATE     = {ALLARME: "#f2c14e", PERSISTENTE: "#c1666b"}


def parse_inject(spec):
    try:
        ch, kind, t0, dur, amp = spec.split(":")
        t0, dur, amp = float(t0), float(dur), float(amp)
    except ValueError:
        raise SystemExit(f"--inject '{spec}': formato canale:tipo:t_inizio:durata:ampiezza")
    if ch not in CHANNELS or kind not in ("step", "ramp", "gain"):
        raise SystemExit(f"--inject '{spec}': canale o tipo non valido")
    if kind == "ramp" and dur <= 0:
        raise SystemExit(f"--inject '{spec}': ramp richiede durata > 0")
    return ch, kind, t0, dur, amp


def apply_injections(sens, injections, hz):
    """Ritorna (sens perturbato, lista di intervalli (t0, t1) in secondi)."""
    sens = sens.copy()
    t = np.arange(len(sens)) / hz
    spans = []
    for ch, kind, t0, dur, amp in injections:
        ci = CHANNELS.index(ch)
        t1 = t[-1] + 1.0 / hz if dur <= 0 else t0 + dur
        inside = (t >= t0) & (t < t1)
        if kind == "step":
            sens[inside, ci] += amp
            spans.append((t0, t1))
        elif kind == "gain":
            sens[inside, ci] *= 1.0 + amp
            spans.append((t0, t1))
        else:  # ramp: sale in [t0, t1) e poi resta
            sens[:, ci] += amp * np.clip((t - t0) / dur, 0.0, 1.0)
            spans.append((t0, float(t[-1])))
    return sens, spans


def style_axis(ax):
    ax.set_facecolor(COL_SURFACE)
    ax.grid(True, color=COL_GRID, linewidth=0.8, zorder=0)
    for sp in ("top", "right"):
        ax.spines[sp].set_visible(False)
    for sp in ("left", "bottom"):
        ax.spines[sp].set_color(COL_BASELINE)
    ax.tick_params(colors=COL_MUTED, labelsize=8)


def shade(ax, t, r, spans, hz):
    """Sfondo: stati ALLARME/PERSISTENTE, intervalli iniettati, riagganci."""
    for st, col in COL_STATE.items():
        on = np.concatenate([[False], r["state"] == st, [False]])
        edges = np.flatnonzero(np.diff(on.astype(int)))
        for a, b in zip(edges[::2], edges[1::2]):
            ax.axvspan(t[a], t[b - 1] + 1.0 / hz, color=col, alpha=0.22, linewidth=0, zorder=1)
    for t0, t1 in spans:
        ax.axvspan(t0, t1, facecolor="none", edgecolor=COL_MUTED, hatch="///",
                   linewidth=0, alpha=0.5, zorder=1)
    for tr in t[r["reanchored"]]:
        ax.axvline(tr, color=COL_MUTED, linewidth=0.9, linestyle=":", zorder=2)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--checkpoint", required=True)
    ap.add_argument("--calib", default=None, help="JSON di calibrate.py")
    ap.add_argument("--dataset_dir", default=DEFAULT_DATASET)
    ap.add_argument("--scaler_path", default=DEFAULT_SCALER)
    ap.add_argument("--device", default="cpu")
    ap.add_argument("--trial", default=None, help="nome/sottostringa/indice. Default: primo di val.")
    ap.add_argument("--inject", action="append", default=[],
                    help="canale:tipo:t_inizio:durata:ampiezza (ripetibile)")
    ap.add_argument("--out", default=os.path.join(SCRIPT_DIR, "detection_out"))
    args = ap.parse_args()

    device = torch.device(args.device)
    FM, ckpt = load_model(args.checkpoint, device)
    calib = Calibration.load(args.calib or default_calib_path(args.checkpoint))
    dataset, val_trials = load_dataset(args.dataset_dir, args.scaler_path, ckpt)
    hz = calib.config.hz

    ti = resolve_trial(dataset, args.trial, val_trials)
    name = dataset.trial_names[ti]
    split = "val" if ti in val_trials else "train"
    if name in calib.meta.get("trials", []):
        print(f"[nota] {name} e' stato usato per calibrare le soglie.")

    cmd, sens_clean = trial_series(dataset, ti)
    sens, spans = apply_injections(sens_clean, [parse_inject(s) for s in args.inject], hz)

    det = PerturbationDetector(FM, calib, H=dataset.h, device=device,
                               norm_stats=ckpt.get("norm_stats"))
    r = replay(det, cmd, sens)
    t = np.arange(len(cmd)) / hz

    # ---- riepilogo ----
    print(f"\nTrial {name} [{split}] | {len(cmd)} campioni ({t[-1]:.1f} s) | "
          f"{len(spans)} perturbazioni iniettate")
    print(f"Riagganci del rollout a t = {[round(float(x), 1) for x in t[r['reanchored']]]} s")
    if not det.events and det.state != ALLARME:
        print("Nessun allarme.")
    for ev in det.events:
        print(f"ALLARME a {ev['t_alarm_s']:.1f} s (canali: {', '.join(ev['trigger'])}) "
              f"-> {ev['classe'].upper()} deciso a {ev['t_decision_s']:.1f} s")
        if "t_end_s" in ev:
            print(f"    finita a {ev['t_end_s']:.1f} s (durata {ev['durata_s']:.1f} s), "
                  f"ritorno automatico a normale")
        elif ev["classe"] == "persistente":
            print("    ancora in corso a fine trial")
        for c, v in ev["channels"].items():
            if v["classe"] != "nessuna":
                print(f"    {c:12s} {v['classe']:12s} inizio {v['inizio']}")
    if det.state == ALLARME:
        print("Trial finito con un allarme non ancora classificato.")
    print(f"Stato finale: {det.state}")

    # ---- figura ----
    out_dir = Path(args.out)
    out_dir.mkdir(parents=True, exist_ok=True)
    fig, axes = plt.subplots(4, 1, figsize=(11, 10), sharex=True)
    fig.patch.set_facecolor(COL_SURFACE)

    for ci, ch in enumerate(CHANNELS):
        ax = axes[ci]
        style_axis(ax)
        shade(ax, t, r, spans, hz)
        sc = dataset.scalers[CH_TO_KEY[ch]]
        real = lambda a: a * float(sc.scale_[0]) + float(sc.mean_[0])
        ax.plot(t, real(r["pred"][:, MR, ci]), color=COL_ROLL, linewidth=1.3, zorder=3,
                label="rollout")
        ax.plot(t, real(r["pred"][:, M1, ci]), color=COL_ONE, linewidth=1.3, zorder=4,
                label="one step")
        ax.plot(t, real(sens[:, ci]), color=COL_SIGNAL, linewidth=1.3, zorder=5,
                label="measured")
        ax.set_ylabel(CH_UNIT[ch], color=COL_TEXT_SEC, fontsize=9)
        ax.set_title(ch, color=COL_TEXT, fontsize=11, fontweight="bold", loc="left")
        ax.legend(loc="upper right", ncol=3, frameon=False, fontsize=8,
                  labelcolor=COL_TEXT_SEC)

    for ax, mi, lab in ((axes[2], MR, "S_R"), (axes[3], M1, "S_1")):
        style_axis(ax)
        shade(ax, t, r, spans, hz)
        for ci, ch in enumerate(CHANNELS):
            ax.plot(t, r["score"][:, mi, ci] / calib.threshold[mi, ci],
                    color=COL_CH[ch], linewidth=1.5, zorder=3, label=ch)
        ax.axhline(1.0, color=COL_TEXT, linewidth=1.0, linestyle="--", zorder=2)
        ax.set_ylim(bottom=0)
        ax.set_ylabel(f"{lab} / threshold", color=COL_TEXT_SEC, fontsize=9)
        ax.set_title(f"{lab} — {'rollout' if mi == MR else 'one-step'} score "
                     f"(dashed line = threshold)",
                     color=COL_TEXT, fontsize=11, fontweight="bold", loc="left")
        ax.legend(loc="upper right", ncol=2, frameon=False, fontsize=8,
                  labelcolor=COL_TEXT_SEC)
    axes[3].set_xlabel("time (s)", color=COL_TEXT_SEC, fontsize=9)

    fig.suptitle(f"Perturbation detector — trial {name} [{split}]   "
                 f"yellow: alarm · red: persistent · hatched: injected · dotted: rollout re-anchor",
                 color=COL_TEXT, fontsize=10, fontweight="bold", x=0.01, ha="left", y=0.995)
    fig.tight_layout(rect=[0, 0, 1, 0.97])
    stem = Path(name).stem + ("_inj" if spans else "")
    fig_path = out_dir / f"detector_{stem}.png"
    fig.savefig(fig_path, dpi=160, facecolor=COL_SURFACE)
    plt.close(fig)
    print(f"Salvato {fig_path}")

    ev_path = out_dir / f"detector_{stem}_events.json"
    with open(ev_path, "w") as f:
        json.dump({"trial": name, "split": split, "inject": args.inject,
                   "reanchor_s": [float(x) for x in t[r["reanchored"]]],
                   "final_state": det.state, "events": det.events}, f, indent=2)
    print(f"Salvato {ev_path}")


if __name__ == "__main__":
    main()