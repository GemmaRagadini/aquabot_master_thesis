"""
Test in loop chiuso (closed-loop rollout) del modello CONGIUNTO (IM + FM).

Idea
----
Le due reti condividono lo STESSO ingresso [C_1:H, S_1:H, ctx] e lo STESSO
scaler. Ad ogni tick:

    buffer [C, S] --> [FM] --> sensori(t+1)
                  |-> [IM] --> comando(t+1)

Entrambe le predizioni (istante t+1) rientrano nei buffer condivisi a fine tick,
e si avanza di un solo passo. Nessun teacher forcing dopo il warmup (tranne il
contesto della diretta, vedi sotto).

Contesto della diretta e dell'inversa
-------------------------------------
Nel modello congiunto ENTRAMBE le reti ricevono il contesto statico
[amp, freq, center, dt] (CTX_DIM=4), ancorato a t (ultimo istante di input),
identico a dataset_joint._build_windows. In closed-loop il contesto e' fornito
"vero" dal trial (teacher forcing SOLO sul contesto): isola l'errore sulla
DINAMICA di sensori/comando dall'errore di stima del regime.
amp/freq/center/dt sono normalizzati con lo scaler condiviso (chiavi
'amp','freq','center','dt').

Warmup
------
I primi H campioni VERI del trial (dal validation split) inizializzano i buffer.
Da li' il rollout e' autoregressivo (tranne il contesto, sempre vero).

Uso
---
python3 src/net/Estimator/checkpoints_joint/closed_loop_test.py 
--checkpoint src/net/Estimator/checkpoints_joint/best_P10.pt
--list_trials  
--trial 
--steps 200
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

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
REPO_ROOT  = os.path.abspath(os.path.join(SCRIPT_DIR, "..", "..", "..", ".."))
sys.path.insert(0, os.path.join(REPO_ROOT, "src"))

from net.Estimator.model   import build_models
from net.Estimator.dataset import FishJointDataset

VAL_FRAC   = 0.2
SPLIT_SEED = 42

# --- palette (coerente con plot_prediction_joint.py) ---
COL_SURFACE  = "#fcfcfb"
COL_TEXT     = "#0b0b0b"
COL_TEXT_SEC = "#52514e"
COL_MUTED    = "#898781"
COL_GRID     = "#e1e0d9"
COL_BASELINE = "#c3c2b7"
COL_SIGNAL   = "#0b0b0b"
COL_MODEL    = "#2a78d6"

CHANNEL_UNIT = {"sensor_diff": "unita' sensore", "current": "mA", "cmd": "rad"}


# ---------------- scaler helpers (single-channel StandardScaler) ----------------
def denorm(scaler, x):
    x = np.asarray(x, dtype=np.float64).reshape(-1, 1)
    return scaler.inverse_transform(x).ravel()

def norm(scaler, x):
    x = np.asarray(x, dtype=np.float64).reshape(-1, 1)
    return scaler.transform(x).ravel()


def dims_from_state(sd):
    """gru_hidden e mlp_hidden dalle shape dei pesi (non salvate nel checkpoint)."""
    gru_hidden = sd["gru.weight_hh_l0"].shape[1]
    mlp_hidden = sd["mlp.0.weight"].shape[0]
    return int(gru_hidden), int(mlp_hidden)


def val_trial_indices(dataset):
    _, val_ds = dataset.split_by_trial(val_frac=VAL_FRAC, seed=SPLIT_SEED)
    idxs = np.unique(dataset.window_trial[np.asarray(val_ds.indices)])
    return set(int(i) for i in idxs)


def resolve_trial(dataset, trial_arg, val_idxs):
    names = dataset.trial_names
    if trial_arg is None:
        if val_idxs:
            idx = int(sorted(val_idxs)[0])
            print(f"Nessun --trial: uso '{names[idx]}' (primo del validation split).")
        else:
            idx = 0
            print(f"Nessun --trial e nessun val split: uso '{names[0]}' (fallback).")
        return idx
    try:
        return int(trial_arg)
    except ValueError:
        pass
    if trial_arg in names:
        return names.index(trial_arg)
    matches = [i for i, n in enumerate(names) if trial_arg in n]
    if len(matches) == 1:
        return matches[0]
    if not matches:
        raise ValueError(f"Nessun trial per '{trial_arg}'. Usa --list_trials.")
    raise ValueError(f"'{trial_arg}' ambiguo: {[names[i] for i in matches]}.")


def build_real_from_dataset(ds, trial_idx):
    """Segnali reali (fisici) del trial dagli episodi gia' parsati: stessa
    calibrazione/offset vista dalle reti. Include i parametri del contesto."""
    ep = ds._episodes[trial_idx]
    return {
        "sensor_diff": np.asarray(ep["sensor_diff_cal"], dtype=np.float64),
        "current":     np.asarray(ep["current"],         dtype=np.float64),
        "cmd":         np.asarray(ep["cmd_servo"],        dtype=np.float64),
        # contesto vero del trial: [amp, freq, center, dt]
        "amp_des":     np.asarray(ep["amp_des"],  dtype=np.float64),
        "freq_des":    np.asarray(ep["freq_des"], dtype=np.float64),
        "center":      np.asarray(ep["center"],   dtype=np.float64),
        "dt":          np.asarray(ep["dt"],       dtype=np.float64),
    }


def build_true_context(real, scalers, t):
    """Contesto 'vero' [amp, freq, center, dt] ancorato all'istante t (ultimo
    input), tutti normalizzati con lo scaler condiviso. Identico all'ancoraggio
    di dataset_joint._build_windows (ctx a i-1)."""
    amp    = float(norm(scalers["amp"],    real["amp_des"][t])[0])
    freq   = float(norm(scalers["freq"],   real["freq_des"][t])[0])
    center = float(norm(scalers["center"], real["center"][t])[0])
    dt     = float(norm(scalers["dt"],     real["dt"][t])[0])
    return np.array([amp, freq, center, dt], dtype=np.float32)


# ------------------------------- rollout -------------------------------
def closed_loop_rollout(IM, FM, scalers, real, h, n_steps, device):
    """Un tick = un avanzamento temporale di uno.

    Entrambe le reti ricevono la finestra condivisa [C, S] (ultimi H comandi e
    sensori) + contesto vero a t. FM -> sensori a t+1, IM -> comando a t+1.
    Le predizioni entrano nei buffer a FINE tick.

    Allineamento: i buffer terminano all'indice h-1 (t=h-1), primo istante
    predetto = h. start = h; la curva vera si allinea con slice(start, start+n).
    """
    sd_r, vf_r, cmd_r = real["sensor_diff"], real["current"], real["cmd"]

    sc_sd  = scalers["sd"]
    sc_vf  = scalers["vf"]
    sc_cmd = scalers["cmd"]

    if len(cmd_r) < h + 1:
        raise ValueError(f"Trial troppo corto per h={h}: {len(cmd_r)} campioni.")

    # warmup: buffer allineati, terminano all'indice h-1
    buf_cmd = list(cmd_r[0:h].astype(np.float64))
    buf_sd  = list(sd_r[0:h].astype(np.float64))
    buf_vf  = list(vf_r[0:h].astype(np.float64))

    pred_sd, pred_vf, pred_cmd = [], [], []

    start = h
    n_steps = min(n_steps, len(cmd_r) - start)

    with torch.no_grad():
        for k in range(n_steps):
            t_last = start + k - 1     # ultimo istante di input (al tick 0: h-1)

            # ingresso condiviso [C, S] normalizzato -> (1, H, 3)
            cmd_n = norm(sc_cmd, buf_cmd)
            sd_n  = norm(sc_sd,  buf_sd)
            vf_n  = norm(sc_vf,  buf_vf)
            seq = torch.tensor(
                np.stack([cmd_n, sd_n, vf_n], axis=1),
                dtype=torch.float32, device=device).reshape(1, h, 3)

            # contesto vero a t
            ctx_vec = build_true_context(real, scalers, t_last)
            ctx = torch.tensor(ctx_vec, dtype=torch.float32, device=device).reshape(1, -1)

            # FM -> sensori a t+1 (primo passo P), IM -> comando a t+1
            pred_sens, _ = FM(seq, ctx)      # (1, P, 2)
            pred_cmd_t, _ = IM(seq, ctx)     # (1, P, 1)
            ps = pred_sens[0, 0, :].cpu().numpy()    # [sd_n, vf_n] a t+1
            pc = float(pred_cmd_t[0, 0, 0].cpu())    # cmd_n a t+1

            sd_next  = float(denorm(sc_sd,  ps[0])[0])
            vf_next  = float(denorm(sc_vf,  ps[1])[0])
            cmd_next = float(denorm(sc_cmd, pc)[0])

            pred_sd.append(sd_next)
            pred_vf.append(vf_next)
            pred_cmd.append(cmd_next)

            # avanzamento di un passo: i buffer scorrono, entrano le predizioni
            buf_cmd.append(cmd_next); buf_cmd.pop(0)
            buf_sd.append(sd_next);   buf_sd.pop(0)
            buf_vf.append(vf_next);   buf_vf.pop(0)

    return {
        "sensor_diff": np.asarray(pred_sd),
        "current":     np.asarray(pred_vf),
        "cmd":         np.asarray(pred_cmd),
        "start":       start,
        "n_steps":     n_steps,
    }


# --------------------------------- plot ---------------------------------
def panel(ax, t, true_real, pred_real, unit, title):
    ax.set_facecolor(COL_SURFACE)
    ax.grid(True, color=COL_GRID, linewidth=0.8, zorder=0)
    for spine in ("top", "right"):
        ax.spines[spine].set_visible(False)
    for spine in ("left", "bottom"):
        ax.spines[spine].set_color(COL_BASELINE)
    ax.plot(t, true_real, color=COL_SIGNAL, linewidth=1.6, zorder=4, label="segnale reale")
    ax.plot(t, pred_real, color=COL_MODEL,  linewidth=1.6, zorder=3, label="rollout closed-loop")
    ax.set_title(title, color=COL_TEXT, fontsize=12, fontweight="bold", loc="left", pad=10)
    ax.set_ylabel(unit, color=COL_TEXT_SEC, fontsize=9)
    ax.tick_params(colors=COL_MUTED, labelsize=8)
    ax.legend(loc="upper right", frameon=False, fontsize=8, labelcolor=COL_TEXT_SEC)


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--checkpoint", default=os.path.join(
        SCRIPT_DIR, "checkpoints_joint", "best.pt"))
    ap.add_argument("--scaler_path", default=os.path.join(
        REPO_ROOT, "src", "net", "scaler", "scalers_joint.pkl"))
    ap.add_argument("--dataset_dir", default=os.path.join(
        REPO_ROOT, "src", "net", "dataset"))
    ap.add_argument("--device", default="cuda" if torch.cuda.is_available() else "cpu")
    ap.add_argument("--trial", default=None,
                    help="nome/sottostringa/indice del trial. Default: primo del val split.")
    ap.add_argument("--steps", type=int, default=None,
                    help="numero di passi del rollout. Default: tutto il trial.")
    ap.add_argument("--list_trials", action="store_true")
    ap.add_argument("--out", default=os.path.join(SCRIPT_DIR, "closed_loop_joint.png"))
    ap.add_argument("--p", type=int, default=None,
                    help="orizzonte P per costruire i target del dataset. Default: "
                         "il P salvato nel checkpoint. Passalo solo per forzare.")
    args = ap.parse_args()

    device = torch.device(args.device)

    # leggo il P dal checkpoint prima di costruire il dataset (target coerenti).
    # Tollerante: se non leggibile (es. --list_trials), uso il default costante.
    P_ckpt = None
    try:
        _ck = torch.load(args.checkpoint, map_location="cpu", weights_only=False)
        P_ckpt = _ck.get("P", None)
    except Exception:
        pass
    P_ds = args.p if args.p is not None else P_ckpt
    if args.p is not None and P_ckpt is not None and args.p != P_ckpt:
        print(f"[avviso] --p={args.p} diverso dal P del checkpoint ({P_ckpt}): "
              f"uso --p={args.p}.", file=sys.stderr)
    ds_kwargs = {"scaler_path": args.scaler_path}
    if P_ds is not None:
        ds_kwargs["p"] = P_ds

    ds = FishJointDataset(args.dataset_dir, **ds_kwargs)
    try:
        val_idxs = val_trial_indices(ds)   # costruisce finestre/scaler
    except Exception as e:
        print(f"[avviso] impossibile costruire il val split ({e}); "
              f"proseguo senza etichette train/val.", file=sys.stderr)
        val_idxs = set()

    if args.list_trials:
        for i, n in enumerate(ds.trial_names):
            tag = ("val" if i in val_idxs else "train") if val_idxs else "?"
            print(f"{i:3d}  [{tag:5s}]  {n}")
        return

    trial_idx  = resolve_trial(ds, args.trial, val_idxs)
    trial_name = ds.trial_names[trial_idx]
    if val_idxs and trial_idx not in val_idxs:
        print(f"[avviso] '{trial_name}' NON e' nel validation split: "
              f"il test non e' out-of-sample.", file=sys.stderr)

    h = ds.h
    scalers = ds.scalers
    for k in ("sd", "vf", "cmd", "amp", "freq", "center", "dt"):
        if k not in scalers:
            raise ValueError(f"scaler condiviso: manca la chiave '{k}' in {args.scaler_path}")

    # --- modello: due reti dal checkpoint, dimensioni dedotte dai pesi ---
    ckpt = torch.load(args.checkpoint, map_location=device, weights_only=False)
    P = ckpt.get("P", 1)
    ctx_dim = ckpt.get("ctx_dim", int(ds.context.shape[-1]))
    gh_im, mh_im = dims_from_state(ckpt["im_state"])
    gh_fm, mh_fm = dims_from_state(ckpt["fm_state"])
    print(f"Dimensioni dal checkpoint: IM gru={gh_im} mlp={mh_im} | FM gru={gh_fm} mlp={mh_fm}")

    IM, FM = build_models(gru_hidden_im=gh_im, mlp_hidden_im=mh_im,
                          gru_hidden_fm=gh_fm, mlp_hidden_fm=mh_fm,
                          p=P, ctx_dim=ctx_dim)
    IM.load_state_dict(ckpt["im_state"]); IM.to(device).eval()
    FM.load_state_dict(ckpt["fm_state"]); FM.to(device).eval()

    real = build_real_from_dataset(ds, trial_idx)

    n_avail = len(real["cmd"]) - h
    n_steps = n_avail if args.steps is None else min(args.steps, n_avail)
    if n_steps <= 0:
        raise ValueError(f"Trial troppo corto per h={h}: {len(real['cmd'])} campioni.")

    print(f"Trial: {trial_name} | campioni={len(real['cmd'])} | h={h} | "
          f"passi rollout={n_steps}")
    print("Modalita': closed-loop completo (FM + IM), contesto vero dal trial")

    pred = closed_loop_rollout(IM, FM, scalers, real, h, n_steps, device)
    start, n = pred["start"], pred["n_steps"]

    sl = slice(start, start + n)
    true = {"sensor_diff": real["sensor_diff"][sl],
            "current":     real["current"][sl],
            "cmd":         real["cmd"][sl]}

    # asse temporale reale dal CSV
    df = pd.read_csv(Path(args.dataset_dir) / trial_name)
    if "t_rel_sec" in df.columns:
        t_full = df["t_rel_sec"].values.astype(np.float64)
        t_axis = t_full[sl] if len(t_full) >= start + n else np.arange(n) / 20.0
        if len(t_axis) != n:
            t_axis = np.arange(n) / 20.0
    else:
        t_axis = np.arange(n) / 20.0

    def rmse(a, b): return float(np.sqrt(np.mean((np.asarray(a) - np.asarray(b)) ** 2)))
    rmse_sd  = rmse(pred["sensor_diff"], true["sensor_diff"])
    rmse_vf  = rmse(pred["current"],     true["current"])
    rmse_cmd = rmse(pred["cmd"],         true["cmd"])
    print(f"RMSE closed-loop  | sensor_diff {rmse_sd:.3f} | current {rmse_vf:.3f} "
          f"| cmd {rmse_cmd:.4f}")

    fig, axes = plt.subplots(3, 1, figsize=(11, 3.4 * 3), sharex=True)
    fig.patch.set_facecolor(COL_SURFACE)
    panel(axes[0], t_axis, true["sensor_diff"], pred["sensor_diff"],
          CHANNEL_UNIT["sensor_diff"],
          f"sensor_diff — RMSE closed-loop {rmse_sd:.2f} {CHANNEL_UNIT['sensor_diff']}")
    panel(axes[1], t_axis, true["current"], pred["current"],
          CHANNEL_UNIT["current"],
          f"current — RMSE closed-loop {rmse_vf:.2f} {CHANNEL_UNIT['current']}")
    panel(axes[2], t_axis, true["cmd"], pred["cmd"],
          CHANNEL_UNIT["cmd"],
          f"comando (tail_target_rad) — RMSE closed-loop {rmse_cmd:.4f} {CHANNEL_UNIT['cmd']}")

    axes[-1].set_xlabel("tempo (s)", color=COL_TEXT_SEC, fontsize=9)
    fig.suptitle(f"Test closed-loop congiunto (FM↔IM) — trial {trial_name} "
                 f"({n} passi)",
                 color=COL_TEXT, fontsize=13, fontweight="bold", x=0.01, ha="left", y=0.997)
    fig.tight_layout(rect=[0, 0, 1, 0.96])
    fig.savefig(args.out, dpi=160, facecolor=COL_SURFACE)
    print(f"Salvato {args.out}")


if __name__ == "__main__":
    main()