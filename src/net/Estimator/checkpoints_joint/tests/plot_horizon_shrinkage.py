"""
Regressione verso la media delle reti con P > 1.

Con P > 1 ogni istante T viene predetto P volte, da P finestre diverse:
    la finestra che finisce in T-1 lo predice a t+1, quella che finisce in T-2
    a t+2, ..., quella che finisce in T-P a t+P.
Piu' la predizione e' lontana, piu' la rete e' incerta; con la loss MSE tende
allora a predire "il valore medio" -> l'ampiezza delle predizioni si schiaccia
verso la media del segnale all'aumentare dell'orizzonte h.

Lo script produce, nella cartella --out:
  slope_all.png            QUANTITATIVO (grafico principale), i 3 canali affiancati:
                           pendenza della regressione pred_h su vero in funzione
                           di h = 1..P (1 = ampiezza piena, 0.5 = meta' delle
                           escursioni dalla media), media +- std sui trial di val.
  slope_<canale>.png       lo stesso, un canale per figura.
  example_<canale>.png     ESEMPIO su un trial (--trial): un picco e una valle
                           "tipici" (95/5 percentile, --pick) e le P predizioni
                           dello stesso istante, con valore vero e media.
  shrinkage.csv            valori numerici (pendenza e rapporto delle std).

Si possono passare piu' checkpoint (es. Supervised P=10 e Combo P=10) per
confrontarli sugli stessi grafici. Checkpoint con P=1 vengono saltati.

Uso (dalla root della repo):
  python3 .../plot_horizon_shrinkage.py \
      --checkpoint .../best_sup_p10.pt .../best_combo_p10.pt \
      --labels "Supervised P=10" "Combo P=10" \
      --trial trial_25.csv --out shrinkage_p10
"""
import argparse
import os
import sys
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import torch


def _find_repo_root():
    """Risale le cartelle finche' trova src/net/Estimator (indipendente da dove
    si trova lo script)."""
    here = Path(os.path.abspath(__file__)).parent
    for q in [here, *here.parents]:
        if (q / "src" / "net" / "Estimator").is_dir():
            return q
    return here


REPO_ROOT = _find_repo_root()
sys.path.insert(0, str(REPO_ROOT / "src"))

from net.Estimator.model   import build_models      # noqa: E402
from net.Estimator.dataset import FishJointDataset  # noqa: E402

VAL_FRAC   = 0.2
SPLIT_SEED = 42

# (nome, gruppo tensori, indice canale, chiave scaler, unita', rete)
CHANNELS = [
    ("sensor_diff", "sens", 0, "sd",  "sensor units", "FM"),
    ("current",     "sens", 1, "vf",  "mA",           "FM"),
    ("tail_target", "cmd",  0, "cmd", "rad",          "IM"),
]

# --- palette (coerente con gli altri script) ---
COL_SURFACE  = "#fcfcfb"
COL_TEXT     = "#0b0b0b"
COL_TEXT_SEC = "#52514e"
COL_MUTED    = "#898781"
COL_GRID     = "#e1e0d9"
COL_BASELINE = "#c3c2b7"
COL_SIGNAL   = "#0b0b0b"
MODEL_COLORS = ["#2a78d6", "#d9822b", "#3a9a5b", "#8a5cc2"]


# ------------------------------------------------------------------ modello/dati
def dims_from_state(sd):
    gru_hidden = sd["gru.weight_hh_l0"].shape[1]
    mlp_hidden = sd["mlp.0.weight"].shape[0]
    return int(gru_hidden), int(mlp_hidden)


def load_model(path, device):
    ckpt = torch.load(path, map_location=device, weights_only=False)
    P = int(ckpt.get("P", 1))
    ctx_static = int(ckpt.get("ctx_static", 3))
    gh_im, mh_im = dims_from_state(ckpt["im_state"])
    gh_fm, mh_fm = dims_from_state(ckpt["fm_state"])
    IM, FM = build_models(gru_hidden_im=gh_im, mlp_hidden_im=mh_im,
                          gru_hidden_fm=gh_fm, mlp_hidden_fm=mh_fm,
                          p=P, ctx_static=ctx_static)
    IM.load_state_dict(ckpt["im_state"]); IM.to(device).eval()
    FM.load_state_dict(ckpt["fm_state"]); FM.to(device).eval()
    return IM, FM, P


def build_dataset(args, P):
    ds = FishJointDataset(args.dataset_dir, p=P, scaler_path=args.scaler_path)
    _, val_ds = ds.split_by_trial(val_frac=VAL_FRAC, seed=SPLIT_SEED)
    val_trials = sorted(set(int(i) for i in np.unique(
        ds.window_trial[np.asarray(val_ds.indices)])))
    return ds, val_trials


def resolve_trial(names, trial_arg):
    if trial_arg in names:
        return names.index(trial_arg)
    matches = [i for i, n in enumerate(names) if trial_arg in n]
    if len(matches) == 1:
        return matches[0]
    if not matches:
        raise ValueError(f"Nessun trial trovato per '{trial_arg}'.")
    raise ValueError(f"'{trial_arg}' ambiguo: {[names[i] for i in matches]}. "
                     f"Usa il nome completo (es. trial_25.csv).")


def predict_trial(ds, IM, FM, device, trial_idx, batch_size=512):
    """Predizioni dirette (open-loop) su tutte le finestre del trial, in scala
    normalizzata. Ritorna {"cmd": (pred, tgt), "sens": (pred, tgt)} con
    pred/tgt di forma (n, P, C)."""
    idxs = np.sort(np.nonzero(ds.window_trial == trial_idx)[0])
    pc_all, ps_all = [], []
    with torch.no_grad():
        for s in range(0, len(idxs), batch_size):
            b = idxs[s:s + batch_size]
            sc = ds.seq_cmd[b].to(device)
            ss = ds.seq_sens[b].to(device)
            cx = ds.context[b].to(device)
            h_im = IM.encode(sc)
            h_fm = FM.encode(ss)
            pc_all.append(IM.decode(h_im, h_fm, cx).cpu())
            ps_all.append(FM.decode(h_fm, h_im, cx).cpu())
    return {
        "cmd":  (torch.cat(pc_all).numpy(), ds.tgt_cmd[idxs].cpu().numpy()),
        "sens": (torch.cat(ps_all).numpy(), ds.tgt_sens[idxs].cpu().numpy()),
    }


# ---------------------------------------------------------------- metriche pure
def shrinkage_metrics(pred, tgt):
    """pred, tgt: (n, P) per un canale. Per ogni orizzonte h ritorna
    std_ratio[h] = std(pred_h)/std(tgt_h) e slope[h] = cov(pred_h,tgt_h)/var(tgt_h).
    Entrambi sono invarianti alla normalizzazione (scaler lineare)."""
    P = pred.shape[1]
    std_ratio = np.full(P, np.nan)
    slope = np.full(P, np.nan)
    for h in range(P):
        p, t = pred[:, h].astype(np.float64), tgt[:, h].astype(np.float64)
        vt = np.var(t)
        if vt <= 0:
            continue
        std_ratio[h] = np.std(p) / np.sqrt(vt)
        slope[h] = np.mean((p - p.mean()) * (t - t.mean())) / vt
    return std_ratio, slope


def per_instant(pred, tgt):
    """Riordina le predizioni per ISTANTE BERSAGLIO.
    pred, tgt: (n, P). La finestra w al passo s predice l'istante locale j = w + s.
    Per ogni j con tutte le P predizioni (j = P-1 .. n-1) ritorna:
      j_idx   (m,)      indici locali degli istanti
      pred_at (m, P)    pred_at[:, s] = predizione di j fatta a t+(s+1)
      true_at (m,)      valore vero in j"""
    n, P = pred.shape
    j_idx = np.arange(P - 1, n)
    pred_at = np.stack([pred[j_idx - s, s] for s in range(P)], axis=1)
    true_at = tgt[j_idx, 0]
    return j_idx, pred_at, true_at


# ----------------------------------------------------------------------- stile
def style_axes(ax):
    ax.set_facecolor(COL_SURFACE)
    ax.grid(True, color=COL_GRID, linewidth=0.8, zorder=0)
    for sp in ("top", "right"):
        ax.spines[sp].set_visible(False)
    for sp in ("left", "bottom"):
        ax.spines[sp].set_color(COL_BASELINE)
    ax.tick_params(colors=COL_MUTED, labelsize=8)


# ------------------------------------------------------------------------ main
def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--checkpoint", nargs="+", required=True,
                    help="uno o piu' checkpoint con P > 1")
    ap.add_argument("--labels", nargs="*", default=None,
                    help="nomi dei modelli in legenda (stesso ordine dei checkpoint)")
    ap.add_argument("--trial", default="trial_25.csv",
                    help="trial per il grafico di esempio (nome completo)")
    ap.add_argument("--dataset_dir", default=str(REPO_ROOT / "src" / "net" / "dataset"))
    ap.add_argument("--scaler_path", default=str(REPO_ROOT / "src" / "net" / "scaler" / "scalers_joint.pkl"))
    ap.add_argument("--device", default="cuda" if torch.cuda.is_available() else "cpu")
    ap.add_argument("--pick", default="typical", choices=["typical", "extreme"],
                    help="esempio: picco/valle 'typical' (95/5 percentile, default) "
                         "o 'extreme' (max/min assoluti, possono essere spike)")
    ap.add_argument("--out", default="horizon_shrinkage",
                    help="CARTELLA di output")
    args = ap.parse_args()

    device = torch.device(args.device)
    labels = args.labels or [Path(c).stem for c in args.checkpoint]
    if len(labels) != len(args.checkpoint):
        ap.error("--labels deve avere lo stesso numero di elementi di --checkpoint")

    out_dir = Path(args.out)
    out_dir.mkdir(parents=True, exist_ok=True)

    datasets = {}   # P -> (dataset, val_trials)
    results = []    # un dict per modello

    for path, label in zip(args.checkpoint, labels):
        IM, FM, P = load_model(path, device)
        if P < 2:
            print(f"[salto] {label}: P={P}, serve P > 1.", file=sys.stderr)
            continue
        if P not in datasets:
            datasets[P] = build_dataset(args, P)
        ds, val_trials = datasets[P]
        print(f"{label}: P={P} | {len(val_trials)} trial di validazione")

        # --- quantitativo: metriche per trial di validazione ---
        per_ch = {ch[0]: {"std_ratio": [], "slope": []} for ch in CHANNELS}
        for ti in val_trials:
            out = predict_trial(ds, IM, FM, device, ti)
            for name, grp, ci, *_ in CHANNELS:
                pred, tgt = out[grp]
                if pred.shape[0] < 5:
                    continue
                sr, sl = shrinkage_metrics(pred[:, :, ci], tgt[:, :, ci])
                per_ch[name]["std_ratio"].append(sr)
                per_ch[name]["slope"].append(sl)

        # --- esempio: trial scelto ---
        example = None
        try:
            ex_idx = resolve_trial(ds.trial_names, args.trial)
            if ex_idx not in val_trials:
                print(f"[avviso] {args.trial} non e' nel validation split.", file=sys.stderr)
            example = (ex_idx, predict_trial(ds, IM, FM, device, ex_idx))
        except ValueError as e:
            print(f"[avviso] esempio non disponibile: {e}", file=sys.stderr)

        results.append({"label": label, "P": P, "ds": ds,
                        "n_trials": len(val_trials), "metrics": per_ch,
                        "example": example})

    if not results:
        print("Nessun checkpoint con P > 1.", file=sys.stderr)
        sys.exit(1)

    # ---------------------------------------------------------------------- CSV
    rows = []
    for r in results:
        for name, *_ in CHANNELS:
            sr = np.array(r["metrics"][name]["std_ratio"])
            sl = np.array(r["metrics"][name]["slope"])
            for i in range(r["P"]):
                rows.append({"model": r["label"], "channel": name, "h": i + 1,
                             "slope_mean":     np.nanmean(sl[:, i]),
                             "slope_std":      np.nanstd(sl[:, i]),
                             "std_ratio_mean": np.nanmean(sr[:, i]),
                             "std_ratio_std":  np.nanstd(sr[:, i]),
                             "n_trials": sl.shape[0]})
    csv_path = out_dir / "shrinkage.csv"
    pd.DataFrame(rows).round(4).to_csv(csv_path, index=False)
    print(f"Salvato {csv_path}")

    # ------------------------------------------- grafico quantitativo: PENDENZA
    P_max = max(r["P"] for r in results)
    n_tr = results[0]["n_trials"]
    nice_name = {"sensor_diff": "Sensor (bending)", "current": "Motor current",
                 "tail_target": "Tail command"}

    def slope_panel(ax, name, net, show_legend):
        style_axes(ax)
        # zona "nessuno schiacciamento" e zona "tutto verso la media"
        ax.axhline(1.0, color=COL_SIGNAL, linewidth=1.2, zorder=1)
        ax.axhline(0.0, color=COL_MUTED, linewidth=1.0, linestyle=":", zorder=1)
        ax.text(P_max + 0.4, 1.035, "full amplitude", va="bottom", ha="right",
                fontsize=8, color=COL_TEXT_SEC)
        ax.text(P_max + 0.4, 0.025, "prediction = mean", va="bottom", ha="right",
                fontsize=8, color=COL_MUTED)
        for k, r in enumerate(results):
            col = MODEL_COLORS[k % len(MODEL_COLORS)]
            h = np.arange(1, r["P"] + 1)
            arr = np.array(r["metrics"][name]["slope"])          # (trial, P)
            mu, sd = np.nanmean(arr, axis=0), np.nanstd(arr, axis=0)
            ax.fill_between(h, mu - sd, mu + sd, color=col, alpha=0.15,
                            linewidth=0, zorder=2)
            ax.plot(h, mu, color=col, marker="o", markersize=4.5,
                    linewidth=2.0, zorder=3, label=r["label"])
            # valori a t+1 e a t+P scritti sul grafico
            ax.annotate(f"{mu[0]:.2f}", (h[0], mu[0]), textcoords="offset points",
                        xytext=(0, 8 if k == 0 else -14), ha="center",
                        fontsize=8, color=col, fontweight="bold")
            ax.annotate(f"{mu[-1]:.2f}", (h[-1], mu[-1]), textcoords="offset points",
                        xytext=(0, 8 if k == 0 else -14), ha="center",
                        fontsize=8, color=col, fontweight="bold")
        ax.set_xlim(0.5, P_max + 0.5)
        ax.set_ylim(-0.05, 1.15)
        ax.set_xticks(range(1, P_max + 1))
        ax.set_xticklabels([f"t+{i}" for i in range(1, P_max + 1)])
        ax.set_xlabel("how far ahead the value is predicted", color=COL_TEXT_SEC, fontsize=9)
        ax.set_title(f"{nice_name.get(name, name)}  [{net}]", color=COL_TEXT,
                     fontsize=11, fontweight="bold", loc="left")
        if show_legend:
            ax.legend(loc="lower left", frameon=False, fontsize=9, labelcolor=COL_TEXT_SEC)

    # figura unica con i 3 canali
    fig, axes = plt.subplots(1, len(CHANNELS), figsize=(15, 4.3), sharey=True)
    fig.patch.set_facecolor(COL_SURFACE)
    for i, (ax, (name, _, _, _, _, net)) in enumerate(zip(axes, CHANNELS)):
        slope_panel(ax, name, net, show_legend=(i == 0))
    axes[0].set_ylabel("regression slope  (predicted vs true)", color=COL_TEXT_SEC, fontsize=9)
    fig.suptitle("Predictions shrink toward the mean as the horizon grows",
                 color=COL_TEXT, fontsize=13, fontweight="bold", x=0.01, ha="left", y=0.99)
    fig.text(0.01, 0.915,
             f"Slope 1 = prediction follows the true excursions from the mean; "
             f"0.5 = only half of them. Validation set, {n_tr} trials, mean ± std.",
             color=COL_TEXT_SEC, fontsize=9, ha="left")
    fig.tight_layout(rect=[0, 0, 1, 0.9])
    path = out_dir / "slope_all.png"
    fig.savefig(path, dpi=160, facecolor=COL_SURFACE)
    plt.close(fig)
    print(f"Salvato {path}")

    # una figura per canale (stesso contenuto, per metterne una sola in slide)
    for name, _, _, _, _, net in CHANNELS:
        fig, ax = plt.subplots(1, 1, figsize=(7.5, 4.3))
        fig.patch.set_facecolor(COL_SURFACE)
        slope_panel(ax, name, net, show_legend=True)
        ax.set_ylabel("regression slope  (predicted vs true)", color=COL_TEXT_SEC, fontsize=9)
        fig.suptitle("Predictions shrink toward the mean as the horizon grows",
                     color=COL_TEXT, fontsize=12, fontweight="bold", x=0.01, ha="left", y=0.99)
        fig.tight_layout(rect=[0, 0, 1, 0.92])
        path = out_dir / f"slope_{name}.png"
        fig.savefig(path, dpi=160, facecolor=COL_SURFACE)
        plt.close(fig)
        print(f"Salvato {path}")

    # --------------------------------------------------------- grafico esempio
    ref = next((r for r in results if r["example"] is not None), None)
    if ref is None:
        print("Nessun esempio disegnato.", file=sys.stderr)
        return

    ds_ref = ref["ds"]
    trial_name = ds_ref.trial_names[ref["example"][0]]
    t_full = None
    csv_trial = Path(args.dataset_dir) / trial_name
    if csv_trial.exists():
        df = pd.read_csv(csv_trial)
        if "t_rel_sec" in df.columns:
            t_full = df["t_rel_sec"].values.astype(np.float64)

    for name, grp, ci, sc_key, unit, net in CHANNELS:
        scaler = ds_ref.scalers[sc_key]
        inv = lambda a: scaler.inverse_transform(np.asarray(a, dtype=np.float64).reshape(-1, 1)).ravel()
        train_mean = float(scaler.mean_[0])

        # picco e valle "tipici": l'istante piu' vicino al 95-esimo / 5-esimo
        # percentile del segnale (evita spike isolati); --pick extreme usa max/min
        pr, tg = ref["example"][1][grp]
        j_ref, _, true_ref = per_instant(pr[:, :, ci], tg[:, :, ci])
        if args.pick == "extreme":
            i_hi, i_lo = int(np.argmax(true_ref)), int(np.argmin(true_ref))
        else:
            q_hi, q_lo = np.percentile(true_ref, 95), np.percentile(true_ref, 5)
            i_hi = int(np.argmin(np.abs(true_ref - q_hi)))
            i_lo = int(np.argmin(np.abs(true_ref - q_lo)))
        picks = [("Peak", int(j_ref[i_hi])), ("Trough", int(j_ref[i_lo]))]

        fig, axes = plt.subplots(1, 2, figsize=(12, 4.3))
        fig.patch.set_facecolor(COL_SURFACE)
        for ax, (kind, j) in zip(axes, picks):
            style_axes(ax)
            true_val = None
            for k, r in enumerate(results):
                if r["example"] is None:
                    continue
                p_m, t_m = r["example"][1][grp]
                j_idx, pred_at, true_at = per_instant(p_m[:, :, ci], t_m[:, :, ci])
                hit = np.nonzero(j_idx == j)[0]
                if len(hit) == 0:
                    continue
                row = hit[0]
                true_val = float(inv(true_at[row])[0])
                h = np.arange(1, r["P"] + 1)
                ax.plot(h, inv(pred_at[row]), color=MODEL_COLORS[k % len(MODEL_COLORS)],
                        marker="o", markersize=5.5, linewidth=2.0, zorder=3, label=r["label"])
            if true_val is not None:
                ax.axhline(true_val, color=COL_SIGNAL, linewidth=1.8, zorder=2, label="true value")
                # freccia "verso la media"
                ax.annotate("", xy=(P_max + 0.3, true_val + 0.35 * (train_mean - true_val)),
                            xytext=(P_max + 0.3, true_val),
                            arrowprops=dict(arrowstyle="->", color=COL_MUTED, lw=1.4))
                ax.text(P_max + 0.45, true_val + 0.18 * (train_mean - true_val), "toward\nthe mean",
                        fontsize=8, color=COL_MUTED, va="center", ha="left")
            ax.axhline(train_mean, color=COL_MUTED, linestyle="--", linewidth=1.3,
                       zorder=2, label="signal mean")

            ax.set_xlim(0.5, P_max + 1.4)
            ax.set_xticks(range(1, P_max + 1))
            ax.set_xticklabels([f"t+{i}" for i in range(1, P_max + 1)])
            ax.set_xlabel("same instant, predicted  1 … 10  steps in advance",
                          color=COL_TEXT_SEC, fontsize=9)
            ax.set_ylabel(unit, color=COL_TEXT_SEC, fontsize=9)
            t_txt = ""
            if t_full is not None and ds_ref.h + j < len(t_full):
                t_txt = f"  (t = {t_full[ds_ref.h + j]:.1f} s)"
            ax.set_title(f"{kind}{t_txt}", color=COL_TEXT, fontsize=11,
                         fontweight="bold", loc="left")
        axes[0].legend(loc="best", frameon=False, fontsize=8, labelcolor=COL_TEXT_SEC)

        fig.suptitle(f"{nice_name.get(name, name)} [{net}] — the further ahead, "
                     f"the closer the prediction to the mean  ·  {trial_name}",
                     color=COL_TEXT, fontsize=12, fontweight="bold", x=0.01, ha="left", y=0.99)
        fig.tight_layout(rect=[0, 0, 1, 0.92])
        path = out_dir / f"example_{name}.png"
        fig.savefig(path, dpi=160, facecolor=COL_SURFACE)
        plt.close(fig)
        print(f"Salvato {path}")

    print(f"Fatto: grafici in {out_dir}/")


if __name__ == "__main__":
    main()