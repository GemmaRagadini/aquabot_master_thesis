"""Funzioni condivise da calibrate.py e run_detector.py: caricamento di modello
e dataset, serie temporale di un trial, replay di un trial nel detector."""
import os
import sys

import numpy as np
import torch

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))


def find_repo_root(start):
    """Risale le cartelle finche' trova quella che contiene src/net."""
    d = os.path.abspath(start)
    while True:
        if os.path.isdir(os.path.join(d, "src", "net")):
            return d
        parent = os.path.dirname(d)
        if parent == d:
            return os.path.abspath(os.path.join(start, "..", "..", "..", ".."))
        d = parent


REPO_ROOT = find_repo_root(SCRIPT_DIR)
sys.path.insert(0, os.path.join(REPO_ROOT, "src"))
sys.path.insert(0, SCRIPT_DIR)

from net.fm.model import build_model          # noqa: E402
from net.dataset import FishJointDataset      # noqa: E402

VAL_FRAC   = 0.2
SPLIT_SEED = 42
CH_TO_KEY  = {"sensor_diff": "sd", "current": "vf"}
CH_UNIT    = {"sensor_diff": "sensor units", "current": "mA"}

DEFAULT_DATASET = os.path.join(REPO_ROOT, "src", "net", "dataset")
DEFAULT_SCALER  = os.path.join(REPO_ROOT, "src", "net", "scaler", "scalers_joint.pkl")


def default_calib_path(checkpoint):
    return os.path.splitext(checkpoint)[0] + "_detector.json"


def load_model(checkpoint, device):
    ckpt = torch.load(checkpoint, map_location=device, weights_only=False)
    if ckpt.get("residual", True):
        raise ValueError(f"{checkpoint}: checkpoint con uscita residua, non supportato.")
    FM = build_model(gru_hidden=ckpt["gru_hidden"], mlp_hidden=ckpt["mlp_hidden"],
                     num_layers=ckpt.get("num_layers", 1))
    FM.load_state_dict(ckpt["fm_state"])
    FM.to(device).eval()
    return FM, ckpt


def load_dataset(dataset_dir, scaler_path, ckpt):
    """Dataset con p=1 e H del checkpoint. Ritorna (dataset, indici dei trial di val)."""
    kwargs = {"scaler_path": scaler_path, "p": 1}
    if ckpt.get("H") is not None:
        kwargs["h"] = int(ckpt["H"])
    dataset = FishJointDataset(dataset_dir, **kwargs)
    _, val_ds = dataset.split_by_trial(val_frac=VAL_FRAC, seed=SPLIT_SEED)
    val_trials = sorted(int(i) for i in np.unique(
        dataset.window_trial[np.asarray(val_ds.indices)]))
    return dataset, val_trials


def trial_series(dataset, trial_idx):
    """Serie NORMALIZZATE dell'intero trial: cmd (n,), sens (n,2).
    Ricostruite dalle finestre p=1: i primi H campioni dalla prima finestra,
    i successivi dai target."""
    idxs = np.sort(np.flatnonzero(dataset.window_trial == trial_idx))
    if len(idxs) == 0:
        raise ValueError(f"Trial {trial_idx} senza finestre.")
    first = int(idxs[0])
    cmd = torch.cat([dataset.seq_cmd[first], dataset.tgt_cmd[idxs][:, 0, :]], dim=0)
    sens = torch.cat([dataset.seq_sens[first], dataset.tgt_sens[idxs][:, 0, :]], dim=0)
    return cmd.cpu().numpy()[:, 0], sens.cpu().numpy()


def resolve_trial(dataset, trial_arg, val_trials):
    names = dataset.trial_names
    if trial_arg is None:
        return val_trials[0] if val_trials else 0
    try:
        return int(trial_arg)
    except ValueError:
        pass
    if trial_arg in names:
        return names.index(trial_arg)
    matches = [i for i, n in enumerate(names) if trial_arg in n]
    if len(matches) == 1:
        return matches[0]
    raise ValueError(f"Trial '{trial_arg}' non trovato o ambiguo: "
                     f"{[names[i] for i in matches]}")


def replay(det, cmd, sens):
    """Fa passare un trial nel detector, campione per campione.
    Ritorna un dict di array lunghi n: state (n,), pred/resid/score (n,2,2)
    [modo, canale], age (n,), alarm/reanchored (n,) bool."""
    det.start()
    outs = [det.step(cmd[i], sens[i]) for i in range(len(cmd))]
    return {
        "state":      np.array([o["state"] for o in outs]),
        "pred":       np.stack([o["pred"] for o in outs]),
        "resid":      np.stack([o["resid"] for o in outs]),
        "score":      np.stack([o["score"] for o in outs]),
        "age":        np.array([o["age"] for o in outs]),
        "alarm":      np.array([o["alarm"] for o in outs]),
        "reanchored": np.array([o["reanchored"] for o in outs]),
    }