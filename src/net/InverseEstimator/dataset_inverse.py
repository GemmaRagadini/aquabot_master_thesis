import ast
import numpy as np
import pandas as pd
import torch
from torch.utils.data import Dataset
from pathlib import Path
from sklearn.preprocessing import StandardScaler

# H: quanti istanti passati sono nel target storia
# a 20Hz, 20 timestep = 1 secondo = un ciclo completo a 1 Hz
H = 20

NEEDED_COLS = ["present_current_ma", "tail_target_rad", "tail_amp_rad", "tail_freq_hz"]

# numero di feature in input alla rete per ogni timestep della finestra.
# INVERSE: [sensor_diff, current]  (i sensori) -> 2
# sensor_mean escluso: speculare all'output della diretta (N_OUTPUTS=2), dove
# sensor_mean e' stato tolto perche' per lo piu' rumore.
# NB: a differenza della rete diretta, qui amp/freq NON entrano in input.
N_INPUT_FEATURES = 2


class FishInverseDataset(Dataset):
    def __init__(self, log_dir: str, h: int = H, scaler_path: str = None):
        """
        Stimatore inverso: input = storia sensoriale (2 canali), output = comando (1).

        scaler_path: percorso del normalizzatore (.pkl).
          - se il file esiste       -> lo carica e lo riusa (nessun refit);
          - se non esiste           -> fitta gli scaler sui CSV e li salva li';
          - se None                 -> fitta e non salva (comportamento volatile).

        NB: questo scaler e' DISTINTO da quello della rete diretta (schema identico,
        file separato). Coerenza garantita dallo stesso metodo di fit globale, senza
        accoppiare i due training su un unico file .pkl.

        Come nella rete diretta, lo scaler NON viene fittato qui nel __init__: se lo
        fittassimo su tutti gli episodi (train + val) le statistiche di
        normalizzazione vedrebbero anche i dati di validation -> data leakage. Il fit
        avviene in prepare(), chiamato da split_by_trial() sui SOLI trial di train.
        """
        self.sequences       = []   # (h, 2)  storia sensoriale in ingresso
        self.targets_history = []   # (h, 1)  comandi passati
        self.targets_future  = []   # (1,)        comando al t+1
        self.labels          = []

        # --- provenienza: da quale trial viene ogni finestra ---
        self.window_trial = []
        self.trial_names  = []

        self.norm_stats = {}
        self.scalers    = {}
        self.h = h

        csv_files = list(Path(log_dir).glob("trial_*.csv"))
        if not csv_files:
            raise FileNotFoundError(f"Nessun csv in {log_dir}")

        print(f"Trovati {len(csv_files)} trial.")

        # --- PASSATA 1: estrai i segnali grezzi (calibrati) da ogni episodio ---
        episodes = []
        for csv_path in csv_files:
            try:
                df = pd.read_csv(csv_path)
                ep = self._extract_signals(df, h, csv_path.name)
                ep["name"] = csv_path.name
                episodes.append(ep)
            except Exception as e:
                print(f"  Skipped {csv_path.name}: {e}")

        if not episodes:
            raise ValueError("Nessun episodio valido dopo il parsing dei CSV.")

        # --- conservo gli episodi grezzi e la loro provenienza ---
        # Lo scaler NON viene piu' fittato qui (vedi docstring): il fit avviene in
        # prepare(), chiamato da split_by_trial() sui SOLI trial di train.
        self._episodes    = episodes
        self._scaler_path = scaler_path
        self.trial_names  = [ep["name"] for ep in episodes]
        self._prepared    = False
        self._prepare_lock = __import__("threading").Lock()
        self._device      = None

        print(f"Dataset grezzo: {len(episodes)} trial letti. "
              f"Scaler e finestre verranno costruiti in split_by_trial() "
              f"(fit solo sul training set).")

    def _finalize_windows(self):
        """Converte in tensori le liste di finestre e controlla i NaN.
        Chiamato al termine di prepare()."""
        self.sequences       = torch.tensor(np.array(self.sequences),       dtype=torch.float32)
        self.targets_history = torch.tensor(np.array(self.targets_history), dtype=torch.float32)
        self.targets_future  = torch.tensor(np.array(self.targets_future),  dtype=torch.float32)
        self.labels          = torch.tensor(np.array(self.labels),          dtype=torch.float32)
        self.window_trial    = np.asarray(self.window_trial, dtype=np.int64)

        # --- guardia finale: nessun NaN/Inf deve arrivare al training ---
        for name, t in [("sequences", self.sequences),
                        ("targets_history", self.targets_history),
                        ("targets_future", self.targets_future)]:
            if not torch.isfinite(t).all():
                raise ValueError(f"NaN/Inf residui in {name}: controlla i CSV con check_nan.py")

        if self._device is not None:
            self._move_tensors(self._device)

        print(f"Dataset: {len(self)} campioni da {len(self._episodes)} trial. "
              f"seq feature/timestep = {self.sequences.shape[-1]}")

    def prepare(self, train_trial_ids):
        """Fitta lo scaler SOLO sui trial di train, poi costruisce tutte le
        finestre (train + val) usando quello scaler.

        train_trial_ids: iterable di indici di trial (posizione in self._episodes)
                         da usare per il fit dello scaler.

        Comportamento scaler_path:
          - file esistente -> carica e riusa (nessun refit);
          - file assente   -> fitta sul train e salva;
          - None           -> fitta sul train e non salva.

        Thread-safe: con Optuna in n_jobs>1 piu' trial chiamano split_by_trial
        (-> prepare) in parallelo sullo stesso dataset. Il lock + double-check
        garantiscono che fit e costruzione finestre avvengano UNA sola volta;
        gli altri thread aspettano e poi trovano _prepared=True.
        """
        if self._prepared:
            return

        with self._prepare_lock:
            # double-check: un altro thread potrebbe aver gia' preparato
            # mentre questo era in attesa del lock
            if self._prepared:
                return
            self._prepare_locked(train_trial_ids)

    def _prepare_locked(self, train_trial_ids):
        train_trial_ids = set(int(i) for i in train_trial_ids)
        train_episodes  = [ep for idx, ep in enumerate(self._episodes)
                           if idx in train_trial_ids]
        if not train_episodes:
            raise ValueError("prepare(): nessun trial di train per fittare lo scaler.")

        scaler_path = self._scaler_path
        if scaler_path is not None and Path(scaler_path).exists():
            self.scalers = self.load_scalers(scaler_path)
            # guardia: chiavi attese per l'inverse (sm non piu' in input)
            needed = {"sd", "cmd", "vf"}
            missing = [k for k in needed if k not in self.scalers]
            if missing:
                raise ValueError(
                    f"Lo scalers.pkl in {scaler_path} non contiene {missing}: e' "
                    f"incompatibile con questa versione dell'inverse. "
                    f"Cancellalo o usa un nuovo --scaler_path per rifittarlo."
                )
            self._sync_norm_stats()
            print(f"Normalizzatore caricato da {scaler_path} (nessun refit).")
        else:
            # FIT SOLO SUL TRAIN: niente leak dai dati di validation
            self._fit_scalers(train_episodes)
            if scaler_path is not None:
                self.save_scalers(scaler_path)
                print(f"Normalizzatore fittato SUL TRAIN e salvato in {scaler_path}.")
            else:
                print("Normalizzatore fittato SUL TRAIN (non salvato).")

        # costruisci le finestre di TUTTI i trial con lo scaler train-only
        self.sequences       = []
        self.targets_history = []
        self.targets_future  = []
        self.labels          = []
        self.window_trial    = []
        for trial_idx, ep in enumerate(self._episodes):
            self._build_windows(ep, self.h, trial_idx)

        self._finalize_windows()
        self._prepared = True

    def to(self, device):
        # Puo' essere chiamato PRIMA di prepare() (quando i tensori non esistono
        # ancora): in quel caso memorizzo il device e spostero' i tensori alla
        # fine di prepare().
        self._device = device
        if self._prepared:
            self._move_tensors(device)
        return self

    def _move_tensors(self, device):
        self.sequences       = self.sequences.to(device)
        self.targets_history = self.targets_history.to(device)
        self.targets_future  = self.targets_future.to(device)
        self.labels          = self.labels.to(device)

    def _parse_sensor_values(self, series: pd.Series):
        def parse_one(s):
            try:
                vals = ast.literal_eval(str(s))
                if len(vals) < 2:
                    return [0.0, 0.0]
                out = [float(v) for v in vals[:2]]
                if not all(np.isfinite(out)):
                    return [np.nan, np.nan]   # marcato, poi interpolato
                return out
            except Exception:
                return [np.nan, np.nan]

        parsed = series.apply(parse_one)
        return np.array(parsed.tolist(), dtype=np.float32)

    def _extract_signals(self, df: pd.DataFrame, h: int, fname: str = ""):
        if len(df) < h + 2:
            raise ValueError(f"impossibile costruire finestre ({len(df)} righe < {h + 2})")
        if len(df) < 50:
            print(f"  [{fname}] attenzione: calibrazione a riposo su {len(df)} campioni (< 50)")

        # --- coercizione numerica + interpolazione dei NaN ---
        df = df.copy()
        for c in NEEDED_COLS:
            if c not in df.columns:
                raise ValueError(f"colonna mancante: {c}")
            df[c] = pd.to_numeric(df[c], errors="coerce")

        n_nan = int(df[NEEDED_COLS].isna().sum().sum())
        if n_nan:
            frac = n_nan / (len(df) * len(NEEDED_COLS))
            if frac > 0.05:
                raise ValueError(f"troppi NaN ({n_nan}, {frac:.1%})")
            print(f"  [{fname}] {n_nan} NaN interpolati")
            df[NEEDED_COLS] = df[NEEDED_COLS].interpolate(limit_direction="both")

        # --- sensori (righe non parsabili -> NaN -> interpolazione) ---
        sensors = self._parse_sensor_values(df["sensor_values"])
        if np.isnan(sensors).any():
            n_bad = int(np.isnan(sensors[:, 0]).sum())
            if n_bad / len(sensors) > 0.05:
                raise ValueError(f"troppe righe sensor_values invalide ({n_bad})")
            for k in range(sensors.shape[1]):
                col = sensors[:, k]
                mask = np.isnan(col)
                col[mask] = np.interp(np.flatnonzero(mask),
                                      np.flatnonzero(~mask), col[~mask])
                sensors[:, k] = col

        sensor_diff = sensors[:, 0] - sensors[:, 1]
        sensor_mean = (sensors[:, 0] + sensors[:, 1]) / 2.0

        # offset a riposo: media dei primi 50 campioni (come fa master_node)
        # sensor_mean viene ancora calcolato e calibrato ma NON entra piu' in input:
        # lasciato qui per non toccare la pipeline di estrazione (resta inutilizzato).
        offset      = sensor_diff[:50].mean()
        offset_mean = sensor_mean[:50].mean()
        sensor_diff_cal = sensor_diff - offset
        sensor_mean_cal = sensor_mean - offset_mean

        current   = df["present_current_ma"].values.astype(np.float32)
        cmd_servo = df["tail_target_rad"].values.astype(np.float32)

        # label invariata (amp, freq desiderati) - restano valori fisici
        amp_des  = df["tail_amp_rad"].values.astype(np.float32)
        freq_des = df["tail_freq_hz"].values.astype(np.float32)

        return {
            "sensor_diff_cal": sensor_diff_cal.astype(np.float32),
            "sensor_mean_cal": sensor_mean_cal.astype(np.float32),
            "cmd_servo":       cmd_servo,
            "current":         current,
            "amp_des":         amp_des,
            "freq_des":        freq_des,
            "offset_diff":     float(offset),
            "offset_mean":     float(offset_mean),
        }

    def _fit_scalers(self, episodes):
        """Fit di uno StandardScaler per ciascun segnale sulle statistiche globali.
        INVERSE: solo i segnali che servono (sd, vf in input; cmd in output).
        sensor_mean escluso dall'input; amp/freq NON normalizzati (solo label fisiche)."""
        all_sd  = np.concatenate([e["sensor_diff_cal"] for e in episodes]).reshape(-1, 1)
        all_cmd = np.concatenate([e["cmd_servo"]       for e in episodes]).reshape(-1, 1)
        all_vf  = np.concatenate([e["current"]         for e in episodes]).reshape(-1, 1)

        self.scalers = {
            "sd":  StandardScaler().fit(all_sd),
            "cmd": StandardScaler().fit(all_cmd),
            "vf":  StandardScaler().fit(all_vf),
        }

        # floor sullo std: evita divisioni per std minuscoli su segnali quasi costanti
        for sc in self.scalers.values():
            sc.scale_ = np.maximum(sc.scale_, 1e-3)

        self._sync_norm_stats()

    def _sync_norm_stats(self):
        self.norm_stats = {
            "sd_mean":  float(self.scalers["sd"].mean_[0]),
            "sd_std":   float(self.scalers["sd"].scale_[0]),
            "cmd_mean": float(self.scalers["cmd"].mean_[0]),
            "cmd_std":  float(self.scalers["cmd"].scale_[0]),
            "vf_mean":  float(self.scalers["vf"].mean_[0]),
            "vf_std":   float(self.scalers["vf"].scale_[0]),
        }

    def _build_windows(self, ep, h, trial_idx):
        sc = self.scalers
        sd_n  = sc["sd"].transform(ep["sensor_diff_cal"].reshape(-1, 1)).ravel()
        cmd_n = sc["cmd"].transform(ep["cmd_servo"].reshape(-1, 1)).ravel()
        vf_n  = sc["vf"].transform(ep["current"].reshape(-1, 1)).ravel()

        # label NON normalizzate (restano i valori fisici come prima)
        amp_des  = ep["amp_des"]
        freq_des = ep["freq_des"]
        n = len(cmd_n)

        # --- finestre scorrevoli ---
        # il loop finisce a n-1 perche' serve il campione t+1 per il target futuro
        for i in range(h, n - 1):
            # input: storia degli ultimi h valori sensoriali  ->  (h, 2)
            # [sensor_diff, current] — sensor_mean escluso (speculare alla diretta)
            seq = np.stack([
                sd_n[i - h:i],
                vf_n[i - h:i],
            ], axis=1)

            # target storia: ultimi h comandi motore  ->  (h, 1)
            target_history = cmd_n[i - h:i].reshape(-1, 1)

            # target futuro: comando al timestep t+1  ->  (1,)
            target_future = np.array([cmd_n[i + 1]], dtype=np.float32)

            # label invariata (amp, freq desiderati)
            label = np.array([amp_des[i], freq_des[i]], dtype=np.float32)

            self.sequences.append(seq)
            self.targets_history.append(target_history)
            self.targets_future.append(target_future)
            self.labels.append(label)
            self.window_trial.append(trial_idx)

    def split_by_trial(self, val_frac=0.2, seed=42):
        """Split train/val a livello di TRIAL, non di finestra.

        Le finestre consecutive si sovrappongono di h-1 timestep: splittare a
        caso sulle finestre (random_split) mette finestre quasi identiche sia
        in train che in val -> data leak, val loss ottimistica e inutile.
        Qui invece si tengono interi trial da un lato o dall'altro, cosi' la
        val misura davvero la generalizzazione a episodi mai visti.

        Speculare a FishDataset.split_by_trial() della rete diretta: lo split si
        decide sui TRIAL prima di costruire le finestre, cosi' lo scaler viene
        fittato (in prepare()) sui soli trial di train.

        Ritorna (train_subset, val_subset).
        """
        from torch.utils.data import Subset

        # Lo split si decide sui TRIAL (indici in self._episodes), prima di
        # costruire le finestre: cosi' possiamo fittare lo scaler sui soli
        # trial di train dentro prepare().
        trial_ids = np.arange(len(self._episodes))
        rng = np.random.default_rng(seed)
        rng.shuffle(trial_ids)

        n_val_trials = max(1, int(round(val_frac * len(trial_ids))))
        val_trials   = set(trial_ids[:n_val_trials].tolist())
        train_trials = [int(t) for t in trial_ids if t not in val_trials]

        if not train_trials or not val_trials:
            raise ValueError(
                f"Split per-trial degenere: train={len(train_trials)} trial, "
                f"val={len(val_trials)} trial. Servono piu' trial (ne hai "
                f"{len(trial_ids)})."
            )

        # fit scaler SOLO sui trial di train + costruzione finestre
        self.prepare(train_trials)

        # ora le finestre esistono: costruisco gli indici train/val
        val_mask  = np.isin(self.window_trial, list(val_trials))
        val_idx   = np.flatnonzero(val_mask)
        train_idx = np.flatnonzero(~val_mask)

        if len(train_idx) == 0 or len(val_idx) == 0:
            raise ValueError(
                f"Split per-trial degenere a livello finestra: "
                f"train={len(train_idx)}, val={len(val_idx)}."
            )

        return Subset(self, train_idx), Subset(self, val_idx)

    def save_scalers(self, path):
        import joblib
        joblib.dump(self.scalers, path)

    @staticmethod
    def load_scalers(path):
        import joblib
        return joblib.load(path)

    def __len__(self):
        return len(self.sequences)

    def __getitem__(self, idx):
        return (
            self.sequences[idx],
            self.targets_history[idx],
            self.targets_future[idx],
            self.labels[idx],
        )


if __name__ == '__main__':
    import sys
    log_dir     = sys.argv[1] if len(sys.argv) > 1 else "../../logs/ds"
    scaler_path = sys.argv[2] if len(sys.argv) > 2 else str(Path(log_dir) / "scalers_inverse.pkl")

    ds = FishInverseDataset(log_dir, scaler_path=scaler_path)
    # le finestre ora vengono costruite in split_by_trial() (scaler fittato
    # solo sul train), quindi lo invoco prima di indicizzare il dataset.
    ds.split_by_trial(val_frac=0.2, seed=42)
    seq, t_hist, t_fut, label = ds[0]
    print(f"seq shape:            {seq.shape}   (h, {N_INPUT_FEATURES}) = storia di [sd, vf]")
    print(f"target_history shape: {t_hist.shape}")   # (20, 1)
    print(f"target_future shape:  {t_fut.shape}")    # (1,)
    print(f"label shape:          {label.shape}")    # (2,)
    print(f"norm_stats keys:      {list(ds.norm_stats.keys())}")