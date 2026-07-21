import argparse
import os
import torch
import torch.nn as nn
import optuna
import random
import numpy as np 

from torch.utils.data import DataLoader, random_split

from model_inverse   import FishInverseEstimator
from dataset_inverse import FishInverseDataset

# Epoche per fase
EPOCHS_PER_PHASE = {1: 30, 2: 30, 3: 50}

# Spazi di ricerca per fase

def suggest_phase1(trial):
    """
    Fase 1 – architettura: solo gru_hidden e mlp_hidden.
    Tutto il resto è fisso ai valori di default.
    Si esaurisce la griglia (4x4 = 16 combinazioni): usa GridSampler.
    """
    gru_hidden = trial.suggest_categorical("gru_hidden", [64, 128, 256, 512])
    mlp_hidden = trial.suggest_categorical("mlp_hidden", [32, 64, 128, 256])
    return dict(
        gru_hidden    = gru_hidden,
        mlp_hidden    = mlp_hidden,
        lr            = 1e-3,      # fisso
        batch_size    = 64,        # fisso
        lambda_future = 1.0,       # fisso
    )


def suggest_phase2(trial, best_arch):
    """
    Fase 2 – esplorazione sparsa: architettura fissata ai valori migliori
    trovati in Fase 1, si esplorano lr, batch_size e lambda_future.
    Optuna usa TPE (Bayesian): non testa tutte le combinazioni ma converge
    sulle più promettenti.
    """
    lr            = trial.suggest_float("lr",            1e-4, 5e-3, log=True)
    batch_size    = trial.suggest_categorical("batch_size",   [32, 64, 128])
    lambda_future = trial.suggest_float("lambda_future", 0.1,  2.0)
    return dict(
        gru_hidden    = best_arch["gru_hidden"],
        mlp_hidden    = best_arch["mlp_hidden"],
        lr            = lr,
        batch_size    = batch_size,
        lambda_future = lambda_future,
    )


def suggest_phase3(trial, best_arch, best_training):
    """
    Fase 3 – tuning finale: tutti i parametri liberi negli intervalli
    validati dalle fasi precedenti. Il sampler TPE sfrutta anche
    la storia dei trial delle fasi precedenti (stesso storage).
    """
    # architettura: ± un livello rispetto al best di fase 1
    arch_choices_gru = _neighbourhood([64, 128, 256, 512], best_arch["gru_hidden"])
    arch_choices_mlp = _neighbourhood([32, 64, 128, 256],  best_arch["mlp_hidden"])
    gru_hidden    = trial.suggest_categorical("gru_hidden",   arch_choices_gru)
    mlp_hidden    = trial.suggest_categorical("mlp_hidden",   arch_choices_mlp)

    # training: range centrato sui best di fase 2
    lr_center     = best_training["lr"]
    lr            = trial.suggest_float("lr",            lr_center / 5, lr_center * 5, log=True)
    batch_size    = trial.suggest_categorical("batch_size",   [32, 64, 128])
    lf_center     = best_training["lambda_future"]
    lambda_future = trial.suggest_float("lambda_future", max(0.05, lf_center - 0.5),
                                                          lf_center + 0.5)
    return dict(
        gru_hidden    = gru_hidden,
        mlp_hidden    = mlp_hidden,
        lr            = lr,
        batch_size    = batch_size,
        lambda_future = lambda_future,
    )


def _neighbourhood(choices, best):
    """Ritorna best + il valore precedente e successivo nella lista."""
    idx = choices.index(best)
    lo  = max(0, idx - 1)
    hi  = min(len(choices) - 1, idx + 1)
    seen, out = set(), []
    for v in choices[lo:hi + 1]:
        if v not in seen:
            seen.add(v)
            out.append(v)
    return out


# Training loop (comune alle tre fasi)

def run_trial(trial, params, dataset, n_epochs):
    gru_hidden    = params["gru_hidden"]
    mlp_hidden    = params["mlp_hidden"]
    lr            = params["lr"]
    batch_size    = params["batch_size"]
    lambda_future = params["lambda_future"]

    # split train/val (stesso seed per riproducibilità tra trial)
    n_val   = int(0.2 * len(dataset))
    n_train = len(dataset) - n_val
    train_ds, val_ds = random_split(
        dataset, [n_train, n_val],
        generator=torch.Generator().manual_seed(42),
    )
    train_loader = DataLoader(train_ds, batch_size=batch_size, shuffle=True)
    val_loader   = DataLoader(val_ds,   batch_size=batch_size)

    model = FishInverseEstimator(
        gru_hidden=gru_hidden,
        mlp_hidden=mlp_hidden,
        h=dataset.h,
    )
    optimizer    = torch.optim.Adam(model.parameters(), lr=lr)
    mse          = nn.MSELoss()
    best_val_loss = float('inf')

    for epoch in range(n_epochs):
        #  training 
        model.train()
        for seq, t_hist, t_fut, _ in train_loader:
            pred_history, pred_future, _ = model(seq)
            loss = mse(pred_history, t_hist) + lambda_future * mse(pred_future, t_fut)
            optimizer.zero_grad()
            loss.backward()
            nn.utils.clip_grad_norm_(model.parameters(), max_norm=1.0)
            optimizer.step()

        #  validation 
        model.eval()
        val_loss = 0.0
        with torch.no_grad():
            for seq, t_hist, t_fut, _ in val_loader:
                pred_history, pred_future, _ = model(seq)
                val_loss += (
                    mse(pred_history, t_hist) + lambda_future * mse(pred_future, t_fut)
                ).item()
        val_loss /= len(val_loader)

        if val_loss < best_val_loss:
            best_val_loss = val_loss

        if epoch % 10 == 0:
            print(f"  Trial {trial.number} | Epoch {epoch:2d} | val {val_loss:.4f}")

        # pruning (non attivo in fase 1 con GridSampler: non ha senso)
        trial.report(val_loss, epoch)
        if trial.should_prune():
            raise optuna.exceptions.TrialPruned()

    return best_val_loss


# Lettura best params da uno studio già completato

def load_best_params(storage, study_name):
    study = optuna.load_study(study_name=study_name, storage=storage)
    return study.best_params


# Main

if __name__ == '__main__':
    
	# fisso i seed 
    random.seed(42)
    np.random.seed(42)
    torch.manual_seed(42)
    
    parser = argparse.ArgumentParser(
        description="Staged Optuna tuning per FishInverseEstimator"
    )
    parser.add_argument('--phase',      type=int, required=True, choices=[1, 2, 3],
                        help="Fase da eseguire: 1, 2 o 3")
    parser.add_argument('--log_dir',    default='../../logs/ds')
    parser.add_argument('--storage',    default='sqlite:///optuna_fish_inverse.db',
                        help="Storage condiviso tra le fasi (consigliato: sqlite)")
    parser.add_argument('--n_trials',   type=int, default=None,
                        help="Numero di trial (default: 16 in fase 1, 30 in fase 2, 50 in fase 3)")
    args = parser.parse_args()

    # n_trials di default per fase
    default_trials = {1: 16, 2: 30, 3: 50}
    n_trials = args.n_trials if args.n_trials is not None else default_trials[args.phase]
    n_epochs  = EPOCHS_PER_PHASE[args.phase]

    print(f"=== FASE {args.phase} | {n_trials} trial | {n_epochs} epoche ===\n")

    print("Caricamento dataset...")
    dataset = FishInverseDataset(args.log_dir)

    study_name = f"fish_inverse_phase{args.phase}"

    
    # Fase 1 – GridSampler: esaurisce tutte le combinazioni archi-
    if args.phase == 1:
        search_space = {
            "gru_hidden": [64, 128, 256, 512],
            "mlp_hidden": [32, 64, 128, 256],
        }
        sampler = optuna.samplers.GridSampler(search_space)
        pruner  = optuna.pruners.NopPruner()   # nessun pruning: la griglia è piccola

        study = optuna.create_study(
            study_name=study_name,
            direction="minimize",
            storage=args.storage,
            load_if_exists=True,
            sampler=sampler,
            pruner=pruner,
        )

        def objective_p1(trial):
            params = suggest_phase1(trial)
            return run_trial(trial, params, dataset, n_epochs)

        study.optimize(objective_p1, n_trials=n_trials, show_progress_bar=True)

    # Fase 2 – TPE sparso su lr, batch_size, lambda_future
    elif args.phase == 2:
        best_arch = load_best_params(args.storage, "fish_inverse_phase1")
        print(f"Architettura fissata da Fase 1: gru_hidden={best_arch['gru_hidden']}, "
              f"mlp_hidden={best_arch['mlp_hidden']}\n")

        study = optuna.create_study(
            study_name=study_name,
            direction="minimize",
            storage=args.storage,
            load_if_exists=True,
            sampler=optuna.samplers.TPESampler(seed=42),
            pruner=optuna.pruners.MedianPruner(n_warmup_steps=5),
        )

        def objective_p2(trial):
            params = suggest_phase2(trial, best_arch)
            return run_trial(trial, params, dataset, n_epochs)

        study.optimize(objective_p2, n_trials=n_trials, show_progress_bar=True)

    # Fase 3 – tuning finale, tutti i parametri
    elif args.phase == 3:
        best_arch     = load_best_params(args.storage, "fish_inverse_phase1")
        best_training = load_best_params(args.storage, "fish_inverse_phase2")
        print(f"Best arch (fase 1):     gru_hidden={best_arch['gru_hidden']}, "
              f"mlp_hidden={best_arch['mlp_hidden']}")
        print(f"Best training (fase 2): lr={best_training['lr']:.2e}, "
              f"batch_size={best_training['batch_size']}, "
              f"lambda_future={best_training['lambda_future']:.3f}\n")

        study = optuna.create_study(
            study_name=study_name,
            direction="minimize",
            storage=args.storage,
            load_if_exists=True,
            sampler=optuna.samplers.TPESampler(seed=42),
            pruner=optuna.pruners.MedianPruner(n_warmup_steps=8),
        )

        def objective_p3(trial):
            params = suggest_phase3(trial, best_arch, best_training)
            return run_trial(trial, params, dataset, n_epochs)

        study.optimize(objective_p3, n_trials=n_trials, show_progress_bar=True)

    # Stampa e salvataggio risultati
    print(f"\n=== Migliori iperparametri – Fase {args.phase} ===")
    for k, v in study.best_params.items():
        print(f"  {k}: {v}")
    print(f"  best val loss: {study.best_value:.4f}")

    os.makedirs("tuning_results", exist_ok=True)
    results_path = f"tuning_results/best_params_inverse_phase{args.phase}.txt"
    with open(results_path, "w") as f:
        # fase 1 – sempre presente
        p1 = load_best_params(args.storage, "fish_inverse_phase1")
        f.write("=== Fase 1 – architettura ===\n")
        for k, v in p1.items():
            f.write(f"  {k}: {v}\n")
        f.write(f"  lr: 1e-3 (fisso)\n")
        f.write(f"  batch_size: 64 (fisso)\n")
        f.write(f"  lambda_future: 1.0 (fisso)\n")
        p1_study = optuna.load_study(study_name="fish_inverse_phase1", storage=args.storage)
        f.write(f"  best val loss: {p1_study.best_value:.4f}\n")

        # fase 2 – se disponibile
        if args.phase >= 2:
            p2 = load_best_params(args.storage, "fish_inverse_phase2")
            p2_study = optuna.load_study(study_name="fish_inverse_phase2", storage=args.storage)
            f.write("\n=== Fase 2 – training ===\n")
            for k, v in p2.items():
                f.write(f"  {k}: {v}\n")
            f.write(f"  best val loss: {p2_study.best_value:.4f}\n")

        # fase 3 – se disponibile
        if args.phase >= 3:
            p3_study = optuna.load_study(study_name="fish_inverse_phase3", storage=args.storage)
            f.write("\n=== Fase 3 – tuning finale ===\n")
            for k, v in study.best_params.items():
                f.write(f"  {k}: {v}\n")
            f.write(f"  best val loss: {p3_study.best_value:.4f}\n")

    print(f"\nRisultati salvati in {results_path}")