import argparse
import math
import os
import random

import numpy as np
import optuna
import torch
import torch.nn as nn
from torch.utils.data import DataLoader, random_split

from net.SensorEstimator.model import FishSensorEstimator
from net.SensorEstimator.dataset import FishDataset

# Epoche per fase
EPOCHS_PER_PHASE = {1: 30, 2: 30, 3: 50}

# device globale (impostato nel main)
DEVICE = torch.device("cpu")


# ---------------------------------------------------------------- search space

def suggest_phase1(trial):
    """Fase 1 - architettura (GridSampler, 4x4 = 16 combinazioni)."""
    gru_hidden = trial.suggest_categorical("gru_hidden", [64, 128, 256, 512])
    mlp_hidden = trial.suggest_categorical("mlp_hidden", [32, 64, 128, 256])
    return dict(
        gru_hidden=gru_hidden,
        mlp_hidden=mlp_hidden,
        lr=1e-3,
        batch_size=64,
        lambda_future=1.0,
    )


def suggest_phase2(trial, best_arch):
    """Fase 2 - lr, batch_size, lambda_future con TPE, architettura fissa."""
    lr            = trial.suggest_float("lr", 1e-4, 5e-3, log=True)
    batch_size    = trial.suggest_categorical("batch_size", [32, 64, 128])
    lambda_future = trial.suggest_float("lambda_future", 0.1, 2.0)
    return dict(
        gru_hidden=best_arch["gru_hidden"],
        mlp_hidden=best_arch["mlp_hidden"],
        lr=lr,
        batch_size=batch_size,
        lambda_future=lambda_future,
    )


def suggest_phase3(trial, best_arch, best_training):
    """Fase 3 - tuning finale attorno ai best delle fasi precedenti."""
    arch_choices_gru = _neighbourhood([64, 128, 256, 512], best_arch["gru_hidden"])
    arch_choices_mlp = _neighbourhood([32, 64, 128, 256],  best_arch["mlp_hidden"])
    gru_hidden = trial.suggest_categorical("gru_hidden", arch_choices_gru)
    mlp_hidden = trial.suggest_categorical("mlp_hidden", arch_choices_mlp)

    lr_center = best_training["lr"]
    lr         = trial.suggest_float("lr", lr_center / 5, lr_center * 5, log=True)
    batch_size = trial.suggest_categorical("batch_size", [32, 64, 128])
    lf_center  = best_training["lambda_future"]
    lambda_future = trial.suggest_float(
        "lambda_future", max(0.05, lf_center - 0.5), lf_center + 0.5
    )
    return dict(
        gru_hidden=gru_hidden,
        mlp_hidden=mlp_hidden,
        lr=lr,
        batch_size=batch_size,
        lambda_future=lambda_future,
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

def run_trial(trial, params, dataset, n_epochs):
    gru_hidden    = params["gru_hidden"]
    mlp_hidden    = params["mlp_hidden"]
    lr            = params["lr"]
    batch_size    = params["batch_size"]
    lambda_future = params["lambda_future"]

    # split train/val (stesso seed -> stesso split per tutti i trial)
    n_val   = int(0.2 * len(dataset))
    n_train = len(dataset) - n_val
    train_ds, val_ds = random_split(
        dataset, [n_train, n_val],
        generator=torch.Generator().manual_seed(42),
    )
    # dataset già in VRAM -> num_workers=0, nessun pin_memory
    train_loader = DataLoader(train_ds, batch_size=batch_size, shuffle=True)
    val_loader   = DataLoader(val_ds,   batch_size=batch_size)

    model = FishSensorEstimator(
        gru_hidden=gru_hidden,
        mlp_hidden=mlp_hidden,
        h=dataset.h,
    ).to(DEVICE)

    optimizer = torch.optim.Adam(model.parameters(), lr=lr)
    mse = nn.MSELoss()
    best_val_loss = float("inf")

    for epoch in range(n_epochs):
        # --- training ---
        model.train()
        for seq, t_hist, t_fut, _ in train_loader:
            pred_history, pred_future, _ = model(seq)
            loss = mse(pred_history, t_hist) + lambda_future * mse(pred_future, t_fut)

            # guardia anti-NaN: se il training esplode, il trial viene potato
            # subito invece di sprecare le epoche rimanenti
            if not torch.isfinite(loss):
                print(f"  Trial {trial.number} | Epoch {epoch:2d} | loss non finita -> pruned")
                raise optuna.exceptions.TrialPruned()

            optimizer.zero_grad()
            loss.backward()
            nn.utils.clip_grad_norm_(model.parameters(), max_norm=1.0)
            optimizer.step()

        # --- validation ---
        model.eval()
        val_loss = 0.0
        with torch.no_grad():
            for seq, t_hist, t_fut, _ in val_loader:
                pred_history, pred_future, _ = model(seq)
                val_loss += (
                    mse(pred_history, t_hist) + lambda_future * mse(pred_future, t_fut)
                ).item()
        val_loss /= len(val_loader)

        if not math.isfinite(val_loss):
            print(f"  Trial {trial.number} | Epoch {epoch:2d} | val nan -> pruned")
            raise optuna.exceptions.TrialPruned()

        best_val_loss = min(best_val_loss, val_loss)
        print(f"  Trial {trial.number} | Epoch {epoch:2d} | val {val_loss:.4f}")

        trial.report(val_loss, epoch)
        if trial.should_prune():
            raise optuna.exceptions.TrialPruned()

    return best_val_loss


# ------------------------------------------------------------------- utilities

def make_storage(url):
    """RDBStorage con timeout lungo: necessario con più worker su sqlite."""
    if url.startswith("sqlite"):
        return optuna.storages.RDBStorage(
            url=url,
            engine_kwargs={"connect_args": {"timeout": 60}},
        )
    return url


def finite_trials(study):
    return [t for t in study.trials
            if t.value is not None and math.isfinite(t.value)]


def load_best_params(storage, study_name):
    study = optuna.load_study(study_name=study_name, storage=storage)
    return study.best_params


def top_phase1_archs(storage, k=2):
    """Le k migliori architetture della fase 1, in ordine."""
    p1 = optuna.load_study(study_name="fish_forward_phase1", storage=storage)
    ranked = sorted(finite_trials(p1), key=lambda t: t.value)
    return [t.params for t in ranked[:k]]


def best_phase2(storage):
    """
    La fase 2 crea studi fish_forward_phase2_arch1/arch2: qui si sceglie
    il miglior trial complessivo e si risale all'architettura associata.
    """
    archs = top_phase1_archs(storage, k=2)
    best_val, best_training, best_arch = float("inf"), None, None
    for rank, arch in enumerate(archs, start=1):
        try:
            s = optuna.load_study(
                study_name=f"fish_forward_phase2_arch{rank}", storage=storage
            )
        except Exception:
            continue
        ft = finite_trials(s)
        if not ft:
            continue
        b = min(ft, key=lambda t: t.value)
        if b.value < best_val:
            best_val, best_training, best_arch = b.value, b.params, arch
    if best_training is None:
        raise RuntimeError("Nessun trial valido trovato negli studi di fase 2")
    return best_arch, best_training, best_val


# ------------------------------------------------------------------------ main

if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Staged Optuna tuning per FishSensorEstimator"
    )
    parser.add_argument("--phase", type=int, required=True, choices=[1, 2, 3])
    parser.add_argument("--log_dir", default="logs/ds")
    parser.add_argument("--storage", default="sqlite:///tuning_results/optuna_fish.db")
    parser.add_argument("--n_trials", type=int, default=None,
                        help="Trial eseguiti DA QUESTO worker (dividi il totale "
                             "per il numero di worker paralleli)")
    parser.add_argument("--device", default="cuda" if torch.cuda.is_available() else "cpu")
    parser.add_argument("--threads", type=int, default=4,
                        help="Thread CPU per worker ")
    parser.add_argument("--seed", type=int, default=None,
                        help="Seed del sampler TPE. Lasciare None con worker "
                             "paralleli, altrimenti campionano tutti gli stessi punti!")
    args = parser.parse_args()

    # seed di torch/numpy per riproducibilità dello split; il sampler TPE
    # invece usa args.seed (None di default, vedi sopra)
    random.seed(42)
    np.random.seed(42)
    torch.manual_seed(42)

    torch.set_num_threads(args.threads)
    DEVICE = torch.device(args.device)
    if DEVICE.type == "cuda":
        torch.backends.cudnn.benchmark = True
    print(f"Device: {DEVICE} | threads: {args.threads}")

    default_trials = {1: 16, 2: 30, 3: 50}
    n_trials = args.n_trials if args.n_trials is not None else default_trials[args.phase]
    n_epochs = EPOCHS_PER_PHASE[args.phase]

    os.makedirs("tuning_results", exist_ok=True)
    storage = make_storage(args.storage)

    print(f"=== FASE {args.phase} | {n_trials} trial (questo worker) | {n_epochs} epoche ===\n")

    print("Caricamento dataset...")
    dataset = FishDataset(args.log_dir).to(DEVICE)   

    study_name = f"fish_forward_phase{args.phase}"

    # ------------------------------------------------ Fase 1 - GridSampler
    if args.phase == 1:
        search_space = {
            "gru_hidden": [64, 128, 256, 512],
            "mlp_hidden": [32, 64, 128, 256],
        }
        sampler = optuna.samplers.GridSampler(search_space, seed=args.seed)
        pruner  = optuna.pruners.NopPruner()

        study = optuna.create_study(
            study_name=study_name,
            direction="minimize",
            storage=storage,
            load_if_exists=True,
            sampler=sampler,
            pruner=pruner,
        )

        def objective_p1(trial):
            params = suggest_phase1(trial)
            return run_trial(trial, params, dataset, n_epochs)

        study.optimize(objective_p1, n_trials=n_trials, n_jobs=4)

    # ------------------------------------- Fase 2 - TPE per le top-2 arch
    elif args.phase == 2:
        archs = top_phase1_archs(storage, k=2)

        for rank, best_arch in enumerate(archs, start=1):
            sub_study_name = f"fish_forward_phase2_arch{rank}"
            print(f"\n--- Fase 2, architettura #{rank}: "
                  f"gru={best_arch['gru_hidden']}, mlp={best_arch['mlp_hidden']} ---\n")

            study = optuna.create_study(
                study_name=sub_study_name,
                direction="minimize",
                storage=storage,
                load_if_exists=True,
                sampler=optuna.samplers.TPESampler(seed=args.seed),
                pruner=optuna.pruners.MedianPruner(n_warmup_steps=5),
            )

            def make_objective(arch):
                def objective(trial):
                    params = suggest_phase2(trial, arch)
                    return run_trial(trial, params, dataset, n_epochs)
                return objective

            study.optimize(make_objective(best_arch), n_trials=n_trials)

    # -------------------------------------------- Fase 3 - tuning finale
    elif args.phase == 3:
        best_arch, best_training, p2_val = best_phase2(storage)
        print(f"Best arch (fase 1/2):   gru_hidden={best_arch['gru_hidden']}, "
              f"mlp_hidden={best_arch['mlp_hidden']}")
        print(f"Best training (fase 2): lr={best_training['lr']:.2e}, "
              f"batch_size={best_training['batch_size']}, "
              f"lambda_future={best_training['lambda_future']:.3f} "
              f"(val {p2_val:.4f})\n")

        study = optuna.create_study(
            study_name=study_name,
            direction="minimize",
            storage=storage,
            load_if_exists=True,
            sampler=optuna.samplers.TPESampler(seed=args.seed),
            pruner=optuna.pruners.MedianPruner(n_warmup_steps=8),
        )

        def objective_p3(trial):
            params = suggest_phase3(trial, best_arch, best_training)
            return run_trial(trial, params, dataset, n_epochs)

        study.optimize(objective_p3, n_trials=n_trials)

    # ------------------------------------------------------------- report
    print(f"\n=== Migliori iperparametri - Fase {args.phase} ===")
    for k, v in study.best_params.items():
        print(f"  {k}: {v}")
    print(f"  best val loss: {study.best_value:.4f}")

    results_path = f"tuning_results/best_params_phase{args.phase}.txt"
    with open(results_path, "w") as f:
        p1_study = optuna.load_study(study_name="fish_forward_phase1", storage=storage)
        f.write("=== Fase 1 - architettura (top 2) ===\n")
        f.write("  (lr: 1e-3, batch_size: 64, lambda_future: 1.0 - fissi)\n")
        p1_top = sorted(finite_trials(p1_study), key=lambda t: t.value)[:2]
        for i, t in enumerate(p1_top):
            f.write(f"\n  #{i+1}  val_loss={t.value:.4f}\n")
            for k, v in t.params.items():
                f.write(f"    {k}: {v}\n")

        if args.phase >= 2:
            f.write("\n=== Fase 2 - training (top 2 per architettura) ===\n")
            for rank in (1, 2):
                try:
                    s = optuna.load_study(
                        study_name=f"fish_forward_phase2_arch{rank}", storage=storage
                    )
                except Exception:
                    continue
                top = sorted(finite_trials(s), key=lambda t: t.value)[:2]
                f.write(f"\n  -- arch #{rank} --\n")
                for i, t in enumerate(top):
                    f.write(f"  #{i+1}  val_loss={t.value:.4f}\n")
                    for k, v in t.params.items():
                        f.write(f"    {k}: {v}\n")

        if args.phase >= 3:
            p3_study = optuna.load_study(study_name="fish_forward_phase3", storage=storage)
            f.write("\n=== Fase 3 - tuning finale (top 2) ===\n")
            p3_top = sorted(finite_trials(p3_study), key=lambda t: t.value)[:2]
            for i, t in enumerate(p3_top):
                f.write(f"\n  #{i+1}  val_loss={t.value:.4f}\n")
                for k, v in t.params.items():
                    f.write(f"    {k}: {v}\n")

    print(f"\nRisultati salvati in {results_path}")