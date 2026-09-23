"""
Tuning Optuna a FASE UNICA del modello congiunto IM + FM.

Tuna ENTRAMBE le reti: con l'hidden incrociato sono accoppiate (gru_hidden_im
entra in FM come cross_hidden e viceversa), quindi ha senso cercarle insieme.

Non duplica il training: chiama direttamente train.train(), con lo stesso
train_mode, la stessa loss e la stessa metrica di selezione del best:
  supervised -> val sup (loss_im + loss_fm su tutto l'orizzonte P)
  rollout    -> val rollout (closed-loop su K passi)
  combo      -> val sup + lambda_roll * val roll, solo a warm-up concluso
Quello che il tuning ottimizza e' quindi esattamente cio' che poi alleni.

Il train_mode (e P, K, lambda_roll, warmup, detach_cross) e' FISSO per studio:
scegli prima la modalita', poi lanci il tuning. Modi diversi hanno metriche non
confrontabili -> studi diversi (il nome dello studio include modo e P).

USO (dalla root della repo):
  python src/net/Estimator/tuning/tune.py --train_mode supervised --p 10 --n_trials 50
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
sys.path.insert(0, ESTIMATOR_DIR)                 # train.py, model.py, dataset.py
sys.path.insert(0, os.path.join(REPO_ROOT, "src"))

import train as T                                  # noqa: E402
from model   import build_models, P as MODEL_P     # noqa: E402
from dataset import FishJointDataset               # noqa: E402

# ---------------------------------------------------------------- search space
GRU_FM   = [64, 128, 256, 384, 512, 768, 1024]
MLP_FM   = [32, 64, 128, 256, 512]
GRU_IM   = [32, 64, 128, 256, 512]
MLP_IM   = [16, 32, 64, 128, 256]
BATCH    = [32, 64, 128, 256]
LR       = (5e-5, 1e-2)
WD       = (1e-6, 1e-1)
DROP_FM  = (0.0, 0.5)
DROP_IM  = (0.0, 0.3)
CLIP     = [0.5, 1.0, 5.0]

# config attuale di train.py: primo trial accodato, cosi' il tuning parte
# almeno da li' (wd_im=0 non e' nel range log -> minimo del range)
CURRENT_DEFAULTS = dict(
    lr=0.0003585794155087849, batch_size=32,
    gru_hidden_im=128, mlp_hidden_im=64, dropout_im=0.0, weight_decay_im=1e-6,
    gru_hidden_fm=256, mlp_hidden_fm=128, dropout_fm=0.10842905375567242,
    weight_decay_fm=2.5314946929205504e-05, clip_norm=1.0,
)


def suggest_params(trial):
    return dict(
        # FM
        gru_hidden_fm   = trial.suggest_categorical("gru_hidden_fm", GRU_FM),
        mlp_hidden_fm   = trial.suggest_categorical("mlp_hidden_fm", MLP_FM),
        dropout_fm      = trial.suggest_float("dropout_fm", *DROP_FM),
        weight_decay_fm = trial.suggest_float("weight_decay_fm", *WD, log=True),
        # IM
        gru_hidden_im   = trial.suggest_categorical("gru_hidden_im", GRU_IM),
        mlp_hidden_im   = trial.suggest_categorical("mlp_hidden_im", MLP_IM),
        dropout_im      = trial.suggest_float("dropout_im", *DROP_IM),
        weight_decay_im = trial.suggest_float("weight_decay_im", *WD, log=True),
        # condivisi
        lr              = trial.suggest_float("lr", *LR, log=True),
        batch_size      = trial.suggest_categorical("batch_size", BATCH),
        clip_norm       = trial.suggest_categorical("clip_norm", CLIP),
    )


# ---------------------------------------------------------------- objective

def make_objective(dataset, args, P, K, device):
    min_epoch = args.roll_warmup if args.train_mode == "combo" else 0

    def objective(trial):
        hp = suggest_params(trial)

        # seed per trial: riproducibile, e diverso tra trial
        seed = 1000 + trial.number
        random.seed(seed); np.random.seed(seed); torch.manual_seed(seed)

        IM, FM = build_models(
            gru_hidden_im=hp["gru_hidden_im"], mlp_hidden_im=hp["mlp_hidden_im"],
            gru_hidden_fm=hp["gru_hidden_fm"], mlp_hidden_fm=hp["mlp_hidden_fm"],
            dropout_im=hp["dropout_im"], dropout_fm=hp["dropout_fm"], p=P)
        IM, FM = IM.to(device), FM.to(device)

        def on_epoch_end(epoch, va_sel, best_val, best_epoch):
            if not math.isfinite(va_sel):
                raise optuna.exceptions.TrialPruned()
            if best_epoch < 0:            # combo in warm-up: nessun best ancora
                return False
            # il pruner vede il best-so-far (coerente col valore restituito)
            trial.report(best_val, epoch)
            if trial.should_prune():
                raise optuna.exceptions.TrialPruned()
            # early stopping: nessun miglioramento da `patience` epoche
            return epoch - best_epoch >= args.patience

        try:
            _, _, hist = T.train(
                IM, FM, dataset,
                epochs=args.max_epochs, lr=hp["lr"], batch_size=hp["batch_size"],
                checkpoint_dir=None, mode=args.train_mode,
                detach_cross=args.detach_cross, rollout_steps=K,
                lambda_roll=args.lambda_roll, roll_warmup=args.roll_warmup,
                weight_decay_im=hp["weight_decay_im"],
                weight_decay_fm=hp["weight_decay_fm"],
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

        # diagnostica all'epoca del best (per leggere IM e FM separatamente)
        trial.set_user_attr("best_epoch", be)
        for k in ("val_sup", "val_roll", "val_im", "val_fm"):
            trial.set_user_attr(k, float(hist[k][be]))
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


def write_report(study, path, args, P, K):
    ranked = sorted(finite_trials(study), key=lambda t: t.value)
    n_pruned = sum(t.state == optuna.trial.TrialState.PRUNED for t in study.trials)
    with open(path, "w") as f:
        f.write(f"=== {study.study_name} | mode={args.train_mode} P={P} K={K} "
                f"detach_cross={args.detach_cross} ===\n")
        f.write(f"{len(ranked)} completati, {n_pruned} pruned\n")
        for i, t in enumerate(ranked[:5]):
            ua = t.user_attrs
            f.write(f"\n  #{i+1}  val_sel={t.value:.5f}  (epoch {ua.get('best_epoch')}, "
                    f"sup {ua.get('val_sup', float('nan')):.5f}, "
                    f"roll {ua.get('val_roll', float('nan')):.5f}, "
                    f"IM@1 {ua.get('val_im', float('nan')):.5f}, "
                    f"FM@1 {ua.get('val_fm', float('nan')):.5f})\n")
            for k, v in t.params.items():
                f.write(f"    {k}: {v}\n")
        if ranked:
            b = ranked[0].params
            f.write("\n=== Comando train.py con il best ===\n")
            f.write(f"python3 src/net/Estimator/train.py --train_mode {args.train_mode} "
                    f"--p {P}" + (f" --rollout_steps {K}" if args.train_mode != "supervised" else "")
                    + (f" --lambda_roll {args.lambda_roll}" if args.train_mode != "supervised" else "")
                    + (f" --roll_warmup {args.roll_warmup}" if args.train_mode == "combo" else "")
                    + (" --detach_cross" if args.detach_cross else "")
                    + "".join(f" --{k} {v}" for k, v in b.items()) + "\n")
        try:
            imp = optuna.importance.get_param_importances(study)
            f.write("\n=== Importanza parametri (fANOVA) ===\n")
            for k, v in imp.items():
                f.write(f"  {k:16s} {v:.3f}\n")
        except Exception as e:
            f.write(f"\n(importanze non calcolabili: {e})\n")


# ------------------------------------------------------------------------ main

if __name__ == "__main__":
    ap = argparse.ArgumentParser(description="Tuning Optuna fase unica IM+FM")
    # --- schema di training: FISSO per studio, stessi significati di train.py ---
    ap.add_argument("--train_mode", default="supervised",
                    choices=["supervised", "rollout", "combo"])
    ap.add_argument("--p", type=int, default=MODEL_P)
    ap.add_argument("--rollout_steps", type=int, default=None)
    ap.add_argument("--lambda_roll", type=float, default=1.0)
    ap.add_argument("--roll_warmup", type=int, default=10)
    ap.add_argument("--detach_cross", action="store_true")
    # --- tuning ---
    ap.add_argument("--n_trials", type=int, default=50, help="trial di QUESTO worker")
    ap.add_argument("--max_epochs", type=int, default=40)
    ap.add_argument("--patience", type=int, default=15,
                    help="early stopping per trial (> patience dello scheduler, 10)")
    ap.add_argument("--study_name", default=None)
    ap.add_argument("--storage", default=f"sqlite:///{os.path.join(SCRIPT_DIR, 'tuning_results', 'optuna_joint.db')}")
    ap.add_argument("--no_warm_start", action="store_true",
                    help="non accodare la config attuale di train.py come primo trial")
    ap.add_argument("--seed", type=int, default=None, help="seed TPE (None con piu' worker)")
    # --- dati / hardware ---
    ap.add_argument("--dataset_dir", default=os.path.join(REPO_ROOT, "src", "net", "dataset"))
    ap.add_argument("--scaler_path", default=os.path.join(REPO_ROOT, "src", "net", "scaler", "scalers_joint.pkl"))
    ap.add_argument("--device", default="cuda" if torch.cuda.is_available() else "cpu")
    ap.add_argument("--threads", type=int, default=4)
    args = ap.parse_args()

    # stessa logica di train.py per K e T
    P = args.p
    uses_roll = args.train_mode in ("rollout", "combo")
    K = args.rollout_steps if args.rollout_steps is not None else max(P, 10)
    T_len = K + P - 1 if uses_roll else P
    if args.train_mode == "combo" and args.max_epochs <= args.roll_warmup + args.patience:
        ap.error("--max_epochs troppo piccolo rispetto a roll_warmup + patience")

    torch.set_num_threads(args.threads)
    device = torch.device(args.device)
    if device.type == "cuda":
        torch.backends.cudnn.benchmark = True
    T.DEVICE = device

    study_name = args.study_name or (
        f"joint_{args.train_mode}_p{P}" + (f"_k{K}" if uses_roll else "")
        + ("_detach" if args.detach_cross else ""))
    os.makedirs(os.path.join(SCRIPT_DIR, "tuning_results"), exist_ok=True)

    print(f"Studio: {study_name} | device {device} | mode={args.train_mode} P={P} "
          f"K={K if uses_roll else '-'} T={T_len}")
    print("Caricamento dataset...")
    dataset = FishJointDataset(args.dataset_dir, p=T_len,
                               scaler_path=args.scaler_path).to(device)

    sampler = optuna.samplers.TPESampler(
        seed=args.seed, multivariate=True, group=True,
        constant_liar=True, n_startup_trials=25)
    warmup_steps = (args.roll_warmup if args.train_mode == "combo" else 0) + 8
    pruner = optuna.pruners.MedianPruner(n_startup_trials=15, n_warmup_steps=warmup_steps)

    study = optuna.create_study(
        study_name=study_name, direction="minimize",
        storage=make_storage(args.storage), load_if_exists=True,
        sampler=sampler, pruner=pruner)

    if not args.no_warm_start and len(study.trials) == 0:
        study.enqueue_trial(CURRENT_DEFAULTS, skip_if_exists=True)
        print("Primo trial = config attuale di train.py")

    study.optimize(make_objective(dataset, args, P, K, device),
                   n_trials=args.n_trials, gc_after_trial=True)

    report = os.path.join(SCRIPT_DIR, "tuning_results", f"best_{study_name}.txt")
    write_report(study, report, args, P, K)
    print(f"\nBest val: {study.best_value:.5f}")
    for k, v in study.best_params.items():
        print(f"  {k}: {v}")
    print(f"Report: {report}")