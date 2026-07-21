"""
Loss per canale sul validation set
Uso:
  python channel_loss.py --checkpoint checkpoints/best.pt --log_dir logs/ds \
      --gru_hidden 512 --mlp_hidden 128
"""
import argparse
import torch
import torch.nn as nn
from torch.utils.data import DataLoader, random_split

from net.SensorEstimator.model import FishSensorEstimator
from net.SensorEstimator.dataset import FishDataset

CHANNELS = ["sensor_diff", "sensor_mean", "current"]

parser = argparse.ArgumentParser()
parser.add_argument("--checkpoint", default="checkpoints/best.pt")
parser.add_argument("--log_dir",    default="logs/ds")
parser.add_argument("--gru_hidden", type=int, default=512)
parser.add_argument("--mlp_hidden", type=int, default=128)
parser.add_argument("--device",     default="cuda" if torch.cuda.is_available() else "cpu")
args = parser.parse_args()

device = torch.device(args.device)

dataset = FishDataset(args.log_dir)

# stesso split del training
n_val   = int(0.2 * len(dataset))
n_train = len(dataset) - n_val
_, val_ds = random_split(
    dataset, [n_train, n_val],
    generator=torch.Generator().manual_seed(42),
)
val_loader = DataLoader(val_ds, batch_size=256)

ckpt = torch.load(args.checkpoint, map_location=device)
model = FishSensorEstimator(gru_hidden=args.gru_hidden,
                            mlp_hidden=args.mlp_hidden, h=dataset.h).to(device)
model.load_state_dict(ckpt["model_state"])
model.eval()

# accumulo errori quadratici per canale, separati per le due teste
se_fut = torch.zeros(3); n_fut = 0
se_his = torch.zeros(3); n_his = 0

with torch.no_grad():
    for seq, t_hist, t_fut, _ in val_loader:
        seq, t_hist, t_fut = seq.to(device), t_hist.to(device), t_fut.to(device)
        pred_history, pred_future, _ = model(seq)

        se_fut += ((pred_future - t_fut) ** 2).sum(dim=0).cpu()
        n_fut  += t_fut.shape[0]

        se_his += ((pred_history - t_hist) ** 2).reshape(-1, 3).sum(dim=0).cpu()
        n_his  += t_hist.shape[0] * t_hist.shape[1]

mse_fut = (se_fut / n_fut).numpy()
mse_his = (se_his / n_his).numpy()

print(f"\n{'canale':<14}{'MSE futuro':>12}{'MSE storia':>12}")
print("-" * 38)
for c in range(3):
    print(f"{CHANNELS[c]:<14}{mse_fut[c]:>12.4f}{mse_his[c]:>12.4f}")
print("-" * 38)
print(f"{'media':<14}{mse_fut.mean():>12.4f}{mse_his.mean():>12.4f}")