"""
Converte MSE/RMSE calcolati in scala normalizzata (z-score, come li produce
FishDataset in dataset.py) in unita' fisiche reali, usando lo scalers.pkl
salvato durante il training.

-----------------
Ogni canale e' normalizzato con uno sklearn StandardScaler:
    x_norm = (x - mean) / std
Quindi ==> 
    RMSE_reale = RMSE_norm * std
    MSE_reale  = MSE_norm  * std**2

Uso
---
1. Modifica il dizionario MSE_NORM qui sotto con i valori della tua tabella
   (o passa --mse-json path/to/file.json con la stessa struttura).

          python convert_mse_to_real_units.py --scaler scalers.pkl

"""
import argparse
import json
import sys
from pathlib import Path

import joblib
import numpy as np

# Mappa: nome canale (come nella tua tabella) -> chiave dello scaler in scalers.pkl
CHANNEL_TO_SCALER_KEY = {
    "sensor_diff": "sd",   # sensor_diff_cal, stesse unita' di sensor_values
    "sensor_mean": "sm",   # sensor_mean_cal
    "current":     "vf",   # present_current_ma -> mA
}

CHANNEL_UNIT = {
    "sensor_diff": "unita' sensore",
    "sensor_mean": "unita' sensore",
    "current":     "mA",
}

MSE_NORM = {
    "sensor_diff": {"storia": 0.0710, "futuro": 0.0378, "persist": 0.1004},
    "current":     {"storia": 0.0538, "futuro": 0.0439, "persist": 0.1472},
}

def load_scalers(path):
    scalers = joblib.load(path)
    missing = [k for k in ("sd", "vf") if k not in scalers]
    if missing:
        raise ValueError(
            f"Lo scalers.pkl in {path} non contiene le chiavi {missing}. "
            f"E' il file giusto/versione giusta? (vedi dataset.py:_fit_scalers)"
        )
    return scalers


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--scaler", required=True, help="path a scalers.pkl")
    ap.add_argument("--mse-json", default=None,
                     help="opzionale: json con la stessa struttura di MSE_NORM, "
                          "per non dover modificare lo script ad ogni run")
    args = ap.parse_args()

    scalers = load_scalers(args.scaler)

    mse_norm = MSE_NORM
    if args.mse_json:
        with open(args.mse_json) as f:
            mse_norm = json.load(f)

    print(f"{'canale':<14}{'metrica':<10}{'MSE norm':>10}{'RMSE norm':>12}{'RMSE reale':>14}   unita'")
    print("-" * 74)

    results = {}
    for ch, metriche in mse_norm.items():
        key = CHANNEL_TO_SCALER_KEY.get(ch)
        if key is None or key not in scalers:
            print(f"[skip] canale '{ch}': nessuno scaler corrispondente (aggiungi la mappatura in "
                  f"CHANNEL_TO_SCALER_KEY)", file=sys.stderr)
            continue
        std = float(scalers[key].scale_[0])
        unit = CHANNEL_UNIT.get(ch, "")
        results[ch] = {}
        for metrica, mse_n in metriche.items():
            rmse_n = np.sqrt(mse_n)
            rmse_real = rmse_n * std
            mse_real = mse_n * std ** 2
            results[ch][metrica] = {"mse_norm": mse_n, "rmse_norm": rmse_n,
                                     "rmse_real": rmse_real, "mse_real": mse_real, "std": std}
            print(f"{ch:<14}{metrica:<10}{mse_n:>10.4f}{rmse_n:>12.4f}{rmse_real:>14.3f}   {unit}")
        print()

    return results


if __name__ == "__main__":
    main()