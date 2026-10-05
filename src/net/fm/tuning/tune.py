"""
Tuning Optuna del FORWARD MODEL a 1 passo (GRU + MLP).

  sens(t) = f(storia cmd+sens fino a t-1, cmd(t))

Non duplica il training: chiama direttamente train_fm.train(), con la stessa
loss (MSE su sens(t)) e la stessa metrica di selezione del best (val MSE).
Quello che il tuning ottimizza e' quindi esattamente cio' che poi alleni.

Diagnostica salvata per trial (all'epoca del best): val per canale
(sd, current) e rapporto val/persistenza (< 1 = meglio di copiare sens(t-1)).

USO (dalla root della repo):
  python src/net/Estimator/tuning/tune_fm.py --n_trials 50
  ./src/net/Estimator/tuning/run_tuning_fm.sh 8 100            # 8 worker paralleli su CPU
"""
import argparse
import math
import os
import random
import sys

import numpy as np
import optuna
import torch

SCRIPT_DIR    = os.path.dirname(os.path.abspath(__file__))
ESTIMATOR_DIR = os.path.abspath(os.path.join(SCRIPT_DIR, ".."))
REPO_ROOT     = os.path.abspath(os.path.join(ESTIMATOR_DIR, "..", "..", ".."))
sys.path.insert(0, ESTIMATOR_DIR)                 # train_fm.py, model_fm.py, dataset.py
sys.path.insert(0, os.path.join(REPO_ROOT, "src"))

import train_fm as T                               # noqa: E402
from model_fm import build_model, H as MODEL_H     # noqa: E402
from dataset  import FishJointDataset              # noqa: E402

# ---------------------------------------------------------------- search space
GRU      = [32, 64, 128, 256]
MLP      = [64, 128, 256, 512]
LAYERS   = [1, 2]
BATCH    = [64, 128, 256]
LR       = (1e-4, 1e-2)
WD       = (1e-6, 1e-1)
DROP     = (0.0, 0.5)
CLIP     = [0.5, 1.0, 5.0]

# config attuale di train_fm.py (run fm_base): primo trial accodato, cosi' il
# tuning parte almeno da li'
CURRENT_DEFAULTS = dict(
    gru_hidden=128, mlp_hidden=256, num_layers=1, dropout=0.1,
    weight_decay=1e-4, lr=1e-3, batch_size=64, clip_norm=1.0,
)


def suggest_params(trial):
    return dict(
        # architettura
        gru_hidden   = trial.suggest_categorical("gru_hidden", GRU),
        mlp_hidden   = trial.suggest_categorical("mlp_hidden", MLP),
        num_layers   = trial.suggest_categorical("num_layers", LAYERS),
        # regolarizzazione
        dropout      = trial.suggest_float("dropout", *DROP),
        weight_decay = trial.suggest_float("weight_decay", *WD, log=True),
        # ottimizzazione
        lr           = trial.suggest_float("lr", *LR, log=True),
        batch_size   = trial.suggest_categorical("batch_size", BATCH),
        clip_norm    = trial.suggest_categorical("clip_norm", CLIP),
    )


# ---------------------------------------------------------------- objective

def make_objective(dataset, args, device):

    def objective(trial):
        hp = suggest_params(trial)

        # seed per trial: riproducibile, e diverso tra trial
        seed = 1000 + trial.number
        random.seed(seed); np.random.seed(seed); torch.manual_seed(seed)

        FM = build_model(gru_hidden=hp["gru_hidden"], mlp_hidden=hp["mlp_hidden"],
                         num_layers=hp["num_layers"], dropout=hp["dropout"]).to(device)

        def on_epoch_end(epoch, va_loss, best_val, best_epoch):
            if not math.isfinite(va_loss):
                raise optuna.exceptions.TrialPruned()
            # il pruner vede il best-so-far (coerente col valore restituito)
            trial.report(best_val, epoch)
            if trial.should_prune():
                raise optuna.exceptions.TrialPruned()
            # early stopping: nessun miglioramento da `patience` epoche
            return epoch - best_epoch >= args.patience

        try:
            _, hist = T.train(
                FM, dataset,
                epochs=args.max_epochs, lr=hp["lr"], batch_size=hp["batch_size"],
                checkpoint_dir=None, weight_decay=hp["weight_decay"],
                clip_norm=hp["clip_norm"],
                save_checkpoints=False, on_epoch_end=on_epoch_end)
        except RuntimeError as e:         # loss non finita (divergenza) o OOM
            print(f"  Trial {trial.number} interrotto: {e}")
            if device.type == "cuda":
                torch.cuda.empty_cache()
            raise optuna.exceptions.TrialPruned()

        be = hist["best_epoch"]
        if be < 0 or not math.isfinite(hist["best_val"]):
            raise optuna.exceptions.TrialPruned()

        # diagnostica all'epoca del best (per leggere i due canali separatamente)
        trial.set_user_attr("best_epoch", be)
        trial.set_user_attr("val_sd",      float(hist["val_sd"][be]))
        trial.set_user_attr("val_current", float(hist["val_current"][be]))
        trial.set_user_attr("persist_ratio",
                            float(hist["best_val"] / hist["persistence_val"]))
        trial.set_user_attr("n_params", int(sum(p.numel() for p in FM.parameters())))
        return hist["best_val"]

    return objective


# ------------------------------------------------------------------- utilities

def make_storage(url):
    if url.startswith("sqlite"):
        return optuna.storages.RDBStorage(
            url=url, engine_kwargs={"connect_args": {"timeout": 60}})
    return url


def finite_trials(study):
    return [t for t in study.trials
            if t.state == optuna.trial.TrialState.COMPLETE
            and t.value is not None and math.isfinite(t.value)]


def train_cmd(params):
    """Comando train_fm.py equivalente ai parametri di un trial."""
    cmd = "python3 src/net/Estimator/train_fm.py"
    for k, v in params.items():
        cmd += f" --{k} {v}"
    return cmd + " --tag fm_tuned"


def write_report(study, path):
    ranked = sorted(finite_trials(study), key=lambda t: t.value)
    n_pruned = sum(t.state == optuna.trial.TrialState.PRUNED for t in study.trials)
    with open(path, "w") as f:
        f.write(f"=== {study.study_name} | forward model 1 passo ===\n")
        f.write(f"{len(ranked)} completati, {n_pruned} pruned\n")
        for i, t in enumerate(ranked[:5]):
            ua = t.user_attrs
            f.write(f"\n  #{i+1}  val={t.value:.5f}  (epoch {ua.get('best_epoch')}, "
                    f"sd {ua.get('val_sd', float('nan')):.5f}, "
                    f"current {ua.get('val_current', float('nan')):.5f}, "
                    f"val/persist {ua.get('persist_ratio', float('nan')):.3f}, "
                    f"params {ua.get('n_params')})\n")
            for k, v in t.params.items():
                f.write(f"    {k}: {v}\n")
        if ranked:
            f.write("\n=== Comando train_fm.py con il best ===\n")
            f.write(train_cmd(ranked[0].params) + "\n")
        try:
            imp = optuna.importance.get_param_importances(study)
            f.write("\n=== Importanza parametri (fANOVA) ===\n")
            for k, v in imp.items():
                f.write(f"  {k:14s} {v:.3f}\n")
        except Exception as e:
            f.write(f"\n(importanze non calcolabili: {e})\n")


# ------------------------------------------------------------------------ main

if __name__ == "__main__":
    ap = argparse.ArgumentParser(description="Tuning Optuna forward model")
    # --- tuning ---
    ap.add_argument("--n_trials", type=int, default=50, help="trial di QUESTO worker")
    ap.add_argument("--max_epochs", type=int, default=60)
    ap.add_argument("--patience", type=int, default=15,
                    help="early stopping per trial (> patience dello scheduler, 10)")
    ap.add_argument("--study_name", default=None)
    ap.add_argument("--storage", default=f"sqlite:///{os.path.join(SCRIPT_DIR, 'tuning_results', 'optuna_fm.db')}")
    ap.add_argument("--no_warm_start", action="store_true",
                    help="non accodare la config attuale di train_fm.py come primo trial")
    ap.add_argument("--seed", type=int, default=None, help="seed TPE (None con piu' worker)")
    # --- dati / hardware ---
    ap.add_argument("--dataset_dir", default=os.path.join(REPO_ROOT, "src", "net", "dataset"))
    ap.add_argument("--scaler_path", default=os.path.join(REPO_ROOT, "src", "net", "scaler", "scalers_joint.pkl"))
    ap.add_argument("--device", default="cpu")
    ap.add_argument("--threads", type=int, default=4)
    args = ap.parse_args()

    torch.set_num_threads(args.threads)
    device = torch.device(args.device)
    if device.type == "cuda":
        torch.backends.cudnn.benchmark = True

    # stesso nome dello studio gia' fatto con uscita diretta: rilanciando si
    # aggiungono trial a quello, senza mescolarsi col vecchio fm_h20_res
    study_name = args.study_name or f"fm_h{MODEL_H}_nores"
    os.makedirs(os.path.join(SCRIPT_DIR, "tuning_results"), exist_ok=True)

    print(f"Studio: {study_name} | device {device} | H={MODEL_H}")
    print("Caricamento dataset (p=1)...")
    dataset = FishJointDataset(args.dataset_dir, h=MODEL_H, p=1,
                               scaler_path=args.scaler_path).to(device)

    sampler = optuna.samplers.TPESampler(
        seed=args.seed, multivariate=True, group=True,
        constant_liar=True, n_startup_trials=25)
    pruner = optuna.pruners.MedianPruner(n_startup_trials=15, n_warmup_steps=8)

    study = optuna.create_study(
        study_name=study_name, direction="minimize",
        storage=make_storage(args.storage), load_if_exists=True,
        sampler=sampler, pruner=pruner)

    if not args.no_warm_start and len(study.trials) == 0:
        study.enqueue_trial(CURRENT_DEFAULTS, skip_if_exists=True)
        print("Primo trial = config attuale di train_fm.py")

    study.optimize(make_objective(dataset, args, device),
                   n_trials=args.n_trials, gc_after_trial=True)

    report = os.path.join(SCRIPT_DIR, "tuning_results", f"best_{study_name}.txt")
    write_report(study, report)
    print(f"\nBest val: {study.best_value:.5f}")
    for k, v in study.best_params.items():
        print(f"  {k}: {v}")
    print(f"Report: {report}")