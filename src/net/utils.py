import numpy as np
from torch.utils.data import Subset

def split_by_trial(dataset, val_frac=0.2, seed=42):
    """Split train/val PER TRIAL: tutte le finestre di uno stesso trial
    finiscono nello stesso lato. Evita il leakage da finestre scorrevoli
    sovrapposte che con random_split cadrebbero a cavallo di train/val."""
    groups = dataset.window_trial              # (N,) id trial per finestra
    unique = np.unique(groups)
    rng = np.random.default_rng(seed)
    rng.shuffle(unique)

    n_val_groups = max(1, int(round(val_frac * len(unique))))
    val_groups = set(unique[:n_val_groups].tolist())

    val_idx   = np.flatnonzero(np.isin(groups, list(val_groups)))
    train_idx = np.flatnonzero(~np.isin(groups, list(val_groups)))
    return Subset(dataset, train_idx.tolist()), Subset(dataset, val_idx.tolist())