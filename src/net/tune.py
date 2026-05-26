import argparse
import os
import torch
import torch.nn as nn
import optuna
from torch.utils.data import DataLoader, random_split

from model   import FishSensorEstimator
from dataset import FishDataset

# epoche per la ricerca
TUNE_EPOCHS = 30


def run_trial(trial, dataset):
      # spazio di ricerca
      gru_hidden    = trial.suggest_categorical("gru_hidden",   [32, 64, 128])
      mlp_hidden    = trial.suggest_categorical("mlp_hidden",   [32, 64, 128])
      lr            = trial.suggest_float("lr",           1e-4, 1e-2, log=True)
      batch_size    = trial.suggest_categorical("batch_size",   [32, 64, 128])
      lambda_future = trial.suggest_float("lambda_future", 0.1 , 2.0)

      # split train/val
      n_val   = int(0.2 * len(dataset))
      n_train = len(dataset) - n_val
      train_ds, val_ds = random_split(dataset, [n_train, n_val])

      train_loader = DataLoader(train_ds, batch_size=batch_size, shuffle=True)
      val_loader   = DataLoader(val_ds,   batch_size=batch_size)

      # modello
      model = FishSensorEstimator(
            gru_hidden=gru_hidden,
            mlp_hidden=mlp_hidden,
            h_out=dataset.h_out,
      )

      optimizer = torch.optim.Adam(model.parameters(), lr=lr)
      mse = nn.MSELoss()
      best_val_loss = float('inf')

      for epoch in range(TUNE_EPOCHS):
            # training
            model.train()
            for seq, t_hist, t_fut, _ in train_loader:
                  pred_history, pred_future, _ = model(seq)
                  loss = mse(pred_history, t_hist) + lambda_future * mse(pred_future, t_fut)
                  optimizer.zero_grad()
                  loss.backward()
                  nn.utils.clip_grad_norm_(model.parameters(), max_norm=1.0)
                  optimizer.step()

            # validation
            model.eval()
            val_loss = 0.0
            with torch.no_grad():
                  for seq, t_hist, t_fut, _ in val_loader:
                        pred_history, pred_future, _ = model(seq)
                        val_loss += (mse(pred_history, t_hist) + lambda_future * mse(pred_future, t_fut)).item()
            val_loss /= len(val_loader)

            if val_loss < best_val_loss:
                  best_val_loss = val_loss
            
            print(f"  Trial {trial.number} | Epoch {epoch:2d} | val {val_loss:.4f}")
            
            # pruning: se la trial è chiaramente peggiore, Optuna la interrompe prima
            trial.report(val_loss, epoch)
            if trial.should_prune():
                  raise optuna.exceptions.TrialPruned()

      return best_val_loss


if __name__ == '__main__':
      parser = argparse.ArgumentParser()
      parser.add_argument('--log_dir',   default='../../logs/ds')
      parser.add_argument('--n_trials',  type=int, default=50)
      parser.add_argument('--study_name', default='fish_forward')
      parser.add_argument('--storage',   default=None,
                              help="es. sqlite:///optuna.db  per persistere i risultati")
      args = parser.parse_args()

      print("Caricamento dataset...")
      dataset = FishDataset(args.log_dir)

      study = optuna.create_study(
            study_name=args.study_name,
            direction="minimize",
            storage=args.storage,
            load_if_exists=True,
            pruner=optuna.pruners.MedianPruner(n_warmup_steps=5),
      )

      study.optimize(
            lambda trial: run_trial(trial, dataset),
            n_trials=args.n_trials,
            show_progress_bar=True,
      )

      print("\n=== Migliori iperparametri ===")
      for k, v in study.best_params.items():
            print(f"  {k}: {v}")
      print(f"  best val loss: {study.best_value:.4f}")

      # salva un riassunto leggibile
      os.makedirs("tuning_results", exist_ok=True)
      results_path = "tuning_results/best_params_forward.txt"
      with open(results_path, "w") as f:
            f.write(f"study: {args.study_name}\n")
            f.write(f"best val loss: {study.best_value:.4f}\n\n")
            for k, v in study.best_params.items():
                  f.write(f"{k}: {v}\n")
      print(f"\nRisultati salvati in {results_path}")