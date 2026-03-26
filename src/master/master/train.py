import argparse
import torch
import torch.nn as nn
from torch.utils.data import DataLoader, random_split

from master.model   import FishStaticNet
from master.dataset import FishDataset


def train(model, dataset, epochs=100, lr=1e-3, lambda_flow=0.3):
    # TODO:
    # 1. split train/val 80-20
    # 2. loop epoche: forward, loss duale (L_cmd + lambda_flow * L_vflow), backward
    # 3. validation
    # 4. learning rate scheduler
    pass


def compute_fisher(model, dataset, n_samples=500):
    # TODO:
    # calcola la matrice di Fisher sui pesi del modello
    # serve per EWC nella fase di adattamento strutturale
    pass


def save_checkpoint(model, fisher, path):
    # TODO:
    # salva model.state_dict(), fisher, theta_star e norm_stats
    pass


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--log_dir',        default='../../logs')
    parser.add_argument('--checkpoint_dir', default='checkpoints/')
    parser.add_argument('--epochs',         type=int,   default=100)
    parser.add_argument('--lr',             type=float, default=1e-3)
    parser.add_argument('--lambda_flow',    type=float, default=0.3)
    args = parser.parse_args()

    # TODO: istanziare dataset, modello, chiamare train(), compute_fisher(), save_checkpoint()