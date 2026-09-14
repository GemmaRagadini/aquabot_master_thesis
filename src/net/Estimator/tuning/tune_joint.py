"""
Staged Optuna tuning per la rete FM (diretta) del modello congiunto.

Perche' solo FM: nel training congiunto IM (inversa, comando) va a zero quasi
subito e generalizza perfetta -> non serve tunarla. FM (diretta, sensori) e' il
collo di bottiglia (overfitting). Qui alleniamo ENTRAMBE le reti insieme (come
in train_joint) ma cerchiamo solo gli iperparametri di FM; IM resta fissa a una
config ragionevole. La metrica ottimizzata e' la val loss della SOLA FM.

Struttura a 3 fasi (come il vecchio tune.py della rete diretta):
  Fase 1 - architettura FM (gru_hidden x mlp_hidden) con GridSampler
  Fase 2 - training (lr, batch, weight_decay, dropout) con TPE sulle top-2 arch
  Fase 3 - raffinamento attorno ai best delle fasi 1/2

"""
import argparse
import math
import os
import random

import numpy as np
import optuna
import torch
import torch.nn as nn
from torch.utils.data import DataLoader

# flat import (come train_joint). In repo: from net.Joint.model import ...
from net.Estimator.model   import build_models
from net.Estimator.dataset import FishJointDataset

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))

# epoche per fase ridotte: la val di FM tocca il minimo dopo ~10 epoche e poi
# overfitta, quindi non serve girarne 50. Il best_val_fm (minimo lungo le
# epoche) e' comunque catturato.
EPOCHS_PER_PHASE = {1: 20, 2: 20, 3: 30}
DEVICE = torch.device("cpu")

# IM fissa: e' gia' a zero con qualunque config ragionevole, non la tuniamo.
IM_ARCH = dict(gru_hidden_im=128, mlp_hidden_im=64, dropout_im=0.0)


# ---------------------------------------------------------------- search space

def suggest_phase1(trial):
    """Fase 1 - architettura FM (GridSampler, 4x4 = 16 combinazioni)."""
    gru_hidden = trial.suggest_categorical("gru_hidden", [64, 128, 256, 512])
    mlp_hidden = trial.suggest_categorical("mlp_hidden", [32, 64, 128, 256])
    return dict(
        gru_hidden=gru_hidden,
        mlp_hidden=mlp_hidden,
        lr=1e-3,
        batch_size=64,
        weight_decay=0.0,
        dropout=0.0,
    )


def suggest_phase2(trial, best_arch):
    """Fase 2 - lr, batch, weight_decay, dropout con TPE; architettura fissa."""
    lr           = trial.suggest_float("lr", 1e-4, 5e-3, log=True)
    batch_size   = trial.suggest_categorical("batch_size", [32, 64, 128])
    weight_decay = trial.suggest_float("weight_decay", 1e-5, 1e-1, log=True)
    dropout      = trial.suggest_float("dropout", 0.0, 0.35)
    return dict(
        gru_hidden=best_arch["gru_hidden"],
        mlp_hidden=best_arch["mlp_hidden"],
        lr=lr,
        batch_size=batch_size,
        weight_decay=weight_decay,
        dropout=dropout,
    )


def suggest_phase3(trial, best_arch, best_training):
    """Fase 3 - raffinamento attorno ai best delle fasi 1/2."""
    arch_choices_gru = _neighbourhood([64, 128, 256, 512], best_arch["gru_hidden"])
    arch_choices_mlp = _neighbourhood([32, 64, 128, 256],  best_arch["mlp_hidden"])
    gru_hidden = trial.suggest_categorical("gru_hidden", arch_choices_gru)
    mlp_hidden = trial.suggest_categorical("mlp_hidden", arch_choices_mlp)

    lr_center = best_training["lr"]
    lr         = trial.suggest_float("lr", lr_center / 5, lr_center * 5, log=True)
    batch_size = trial.suggest_categorical("batch_size", [32, 64, 128])

    wd_center = best_training.get("weight_decay", 1e-4)
    wd_lo = max(1e-5, wd_center / 5)
    wd_hi = min(1e-1, wd_center * 5)
    if wd_lo >= wd_hi:
        wd_lo, wd_hi = 1e-5, 1e-1
    weight_decay = trial.suggest_float("weight_decay", wd_lo, wd_hi, log=True)

    do_center = best_training.get("dropout", 0.1)
    do_lo = max(0.0,  do_center - 0.15)
    do_hi = min(0.35, do_center + 0.15)
    dropout = trial.suggest_float("dropout", do_lo, do_hi)

    return dict(
        gru_hidden=gru_hidden,
        mlp_hidden=mlp_hidden,
        lr=lr,
        batch_size=batch_size,
        weight_decay=weight_decay,
        dropout=dropout,
    )


def _neighbourhood(choices, best):
    idx = choices.index(best)
    lo = max(0, idx - 1)
    hi = min(len(choices) - 1, idx + 1)
    seen, out = set(), []
    for v in choices[lo:hi + 1]:
        if v not in seen:
            seen.add(v)
            out.append(v)
    return out


# -------------------------------------------------------------- training loop

def run_trial(trial, params, dataset, n_epochs, ctx_dim):
    # split a livello di trial, stesso seed -> confrontabile tra i trial Optuna
    train_ds, val_ds = dataset.split_by_trial(val_frac=0.2, seed=42)
    train_loader = DataLoader(train_ds, batch_size=params["batch_size"], shuffle=True)
    val_loader   = DataLoader(val_ds,   batch_size=params["batch_size"])

    # IM fissa + FM con gli iperparametri del trial
    IM, FM = build_models(
        gru_hidden_fm=params["gru_hidden"],
        mlp_hidden_fm=params["mlp_hidden"],
        dropout_fm=params["dropout"],
        ctx_dim=ctx_dim,
        **IM_ARCH,
    )
    IM.to(DEVICE); FM.to(DEVICE)

    # un solo optimizer su entrambe (come train_joint), weight_decay applicato a
    # tutti i parametri; IM e' comunque piccola e gia' risolta.
    optimizer = torch.optim.Adam(
        list(IM.parameters()) + list(FM.parameters()),
        lr=params["lr"], weight_decay=params["weight_decay"])
    mse = nn.MSELoss()
    best_val_fm = float("inf")

    for epoch in range(n_epochs):
        IM.train(); FM.train()
        for seq_cmd, seq_sens, ctx, tgt_cmd, tgt_sens, _ in train_loader:
            seq = torch.cat([seq_cmd, seq_sens], dim=-1)
            pred_cmd,  _ = IM(seq, ctx)
            pred_sens, _ = FM(seq, ctx)
            loss = mse(pred_cmd, tgt_cmd) + mse(pred_sens, tgt_sens)

            if not torch.isfinite(loss):
                print(f"  Trial {trial.number} | Epoch {epoch:2d} | loss non finita -> pruned")
                raise optuna.exceptions.TrialPruned()

            optimizer.zero_grad()
            loss.backward()
            nn.utils.clip_grad_norm_(list(IM.parameters()) + list(FM.parameters()), max_norm=1.0)
            optimizer.step()

        # --- validation: metrica = SOLO FM ---
        IM.eval(); FM.eval()
        val_fm = 0.0
        with torch.no_grad():
            for seq_cmd, seq_sens, ctx, tgt_cmd, tgt_sens, _ in val_loader:
                seq = torch.cat([seq_cmd, seq_sens], dim=-1)
                pred_sens, _ = FM(seq, ctx)
                val_fm += mse(pred_sens, tgt_sens).item()
        val_fm /= len(val_loader)

        if not math.isfinite(val_fm):
            print(f"  Trial {trial.number} | Epoch {epoch:2d} | val nan -> pruned")
            raise optuna.exceptions.TrialPruned()

        best_val_fm = min(best_val_fm, val_fm)
        print(f"  Trial {trial.number} | Epoch {epoch:2d} | val_FM {val_fm:.4f}")

        trial.report(val_fm, epoch)
        if trial.should_prune():
            raise optuna.exceptions.TrialPruned()

    return best_val_fm


# ------------------------------------------------------------------- utilities

def make_storage(url):
    if url.startswith("sqlite"):
        return optuna.storages.RDBStorage(
            url=url, engine_kwargs={"connect_args": {"timeout": 60}})
    return url


def finite_trials(study):
    return [t for t in study.trials
            if t.value is not None and math.isfinite(t.value)]


def top_phase1_archs(storage, k=2):
    p1 = optuna.load_study(study_name="fish_fm_phase1", storage=storage)
    ranked = sorted(finite_trials(p1), key=lambda t: t.value)
    return [t.params for t in ranked[:k]]


def best_phase2(storage):
    archs = top_phase1_archs(storage, k=2)
    best_val, best_training, best_arch = float("inf"), None, None
    for rank, arch in enumerate(archs, start=1):
        try:
            s = optuna.load_study(study_name=f"fish_fm_phase2_arch{rank}", storage=storage)
        except Exception:
            continue
        ft = finite_trials(s)
        if not ft:
            continue
        b = min(ft, key=lambda t: t.value)
        if b.value < best_val:
            best_val, best_training, best_arch = b.value, b.params, arch
    if best_training is None:
        raise RuntimeError("Nessun trial valido negli studi di fase 2")
    return best_arch, best_training, best_val


# ------------------------------------------------------------------------ main

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Staged Optuna tuning per FM (rete diretta congiunta)")
    parser.add_argument("--phase", type=int, required=True, choices=[1, 2, 3])
    parser.add_argument("--dataset_dir", default="./src/net/dataset")
    parser.add_argument("--scaler_path",
                        default=os.path.join(SCRIPT_DIR, "..", "..", "..", "..",
                                             "src", "net", "scaler", "scalers_joint.pkl"))
    parser.add_argument("--storage", default=f"sqlite:///{os.path.join(SCRIPT_DIR, 'tuning_results', 'optuna_fm.db')}")
    parser.add_argument("--n_trials", type=int, default=None)
    parser.add_argument("--device", default="cuda" if torch.cuda.is_available() else "cpu")
    parser.add_argument("--threads", type=int, default=4)
    parser.add_argument("--seed", type=int, default=None,
                        help="Seed TPE. None con worker paralleli (altrimenti campionano uguale).")
    args = parser.parse_args()

    random.seed(42); np.random.seed(42); torch.manual_seed(42)
    torch.set_num_threads(args.threads)
    DEVICE = torch.device(args.device)
    if DEVICE.type == "cuda":
        torch.backends.cudnn.benchmark = True
    print(f"Device: {DEVICE} | threads: {args.threads}")
    print("Tuning della SOLA FM (IM fissa). Metrica: val loss di FM.")

    default_trials = {1: 16, 2: 30, 3: 50}
    n_trials = args.n_trials if args.n_trials is not None else default_trials[args.phase]
    n_epochs = EPOCHS_PER_PHASE[args.phase]

    os.makedirs(os.path.join(SCRIPT_DIR, "tuning_results"), exist_ok=True)
    storage = make_storage(args.storage)

    print(f"=== FASE {args.phase} | {n_trials} trial (questo worker) | {n_epochs} epoche ===\n")
    print("Caricamento dataset...")
    dataset = FishJointDataset(args.dataset_dir, scaler_path=args.scaler_path).to(DEVICE)
    ctx_dim = None  # letto dopo lo split, dal tensore context
    # forziamo la costruzione delle finestre una volta per leggere ctx_dim
    dataset.split_by_trial(val_frac=0.2, seed=42)
    ctx_dim = int(dataset.context.shape[-1])
    print(f"ctx_dim: {ctx_dim}")

    study_name = f"fish_fm_phase{args.phase}"

    if args.phase == 1:
        search_space = {"gru_hidden": [64, 128, 256, 512],
                        "mlp_hidden": [32, 64, 128, 256]}
        sampler = optuna.samplers.GridSampler(search_space, seed=args.seed)
        study = optuna.create_study(
            study_name=study_name, direction="minimize", storage=storage,
            load_if_exists=True, sampler=sampler, pruner=optuna.pruners.NopPruner())

        def objective_p1(trial):
            return run_trial(trial, suggest_phase1(trial), dataset, n_epochs, ctx_dim)
        study.optimize(objective_p1, n_trials=n_trials, n_jobs=4)

    elif args.phase == 2:
        archs = top_phase1_archs(storage, k=2)
        for rank, best_arch in enumerate(archs, start=1):
            sub = f"fish_fm_phase2_arch{rank}"
            print(f"\n--- Fase 2, arch #{rank}: gru={best_arch['gru_hidden']}, mlp={best_arch['mlp_hidden']} ---\n")
            study = optuna.create_study(
                study_name=sub, direction="minimize", storage=storage,
                load_if_exists=True, sampler=optuna.samplers.TPESampler(seed=args.seed),
                pruner=optuna.pruners.MedianPruner(n_warmup_steps=5))

            def make_obj(arch):
                def obj(trial):
                    return run_trial(trial, suggest_phase2(trial, arch), dataset, n_epochs, ctx_dim)
                return obj
            study.optimize(make_obj(best_arch), n_trials=n_trials)

    elif args.phase == 3:
        best_arch, best_training, p2_val = best_phase2(storage)
        print(f"Best arch: gru={best_arch['gru_hidden']}, mlp={best_arch['mlp_hidden']}")
        print(f"Best training: lr={best_training['lr']:.2e}, batch={best_training['batch_size']}, "
              f"wd={best_training['weight_decay']:.2e}, dropout={best_training['dropout']:.2f} (val {p2_val:.4f})\n")
        study = optuna.create_study(
            study_name=study_name, direction="minimize", storage=storage,
            load_if_exists=True, sampler=optuna.samplers.TPESampler(seed=args.seed),
            pruner=optuna.pruners.MedianPruner(n_warmup_steps=8))

        def objective_p3(trial):
            return run_trial(trial, suggest_phase3(trial, best_arch, best_training), dataset, n_epochs, ctx_dim)
        study.optimize(objective_p3, n_trials=n_trials)

    print(f"\n=== Migliori iperparametri FM - Fase {args.phase} ===")
    for k, v in study.best_params.items():
        print(f"  {k}: {v}")
    print(f"  best val_FM: {study.best_value:.4f}")

    results_path = os.path.join(SCRIPT_DIR, "tuning_results", f"best_fm_phase{args.phase}.txt")
    with open(results_path, "w") as f:
        p1 = optuna.load_study(study_name="fish_fm_phase1", storage=storage)
        f.write("=== Fase 1 - architettura FM (top 2) ===\n  (lr 1e-3, batch 64, wd 0, dropout 0 - fissi)\n")
        for i, t in enumerate(sorted(finite_trials(p1), key=lambda t: t.value)[:2]):
            f.write(f"\n  #{i+1}  val_FM={t.value:.4f}\n")
            for k, v in t.params.items():
                f.write(f"    {k}: {v}\n")
        if args.phase >= 2:
            f.write("\n=== Fase 2 - training (top 2 per arch) ===\n")
            for rank in (1, 2):
                try:
                    s = optuna.load_study(study_name=f"fish_fm_phase2_arch{rank}", storage=storage)
                except Exception:
                    continue
                f.write(f"\n  -- arch #{rank} --\n")
                for i, t in enumerate(sorted(finite_trials(s), key=lambda t: t.value)[:2]):
                    f.write(f"  #{i+1}  val_FM={t.value:.4f}\n")
                    for k, v in t.params.items():
                        f.write(f"    {k}: {v}\n")
        if args.phase >= 3:
            p3 = optuna.load_study(study_name="fish_fm_phase3", storage=storage)
            f.write("\n=== Fase 3 - tuning finale (top 2) ===\n")
            for i, t in enumerate(sorted(finite_trials(p3), key=lambda t: t.value)[:2]):
                f.write(f"\n  #{i+1}  val_FM={t.value:.4f}\n")
                for k, v in t.params.items():
                    f.write(f"    {k}: {v}\n")

    print(f"\nRisultati salvati in {results_path}")