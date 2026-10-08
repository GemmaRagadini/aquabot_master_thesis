import ast
import csv
import os
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from itertools import cycle

LABEL_SIZE  = 14
TITLE_SIZE  = 16
LEGEND_SIZE = 12
TICK_SIZE   = 12

# Ciclo di colori usato per distinguere i vari trial nei confronti.
COMPARE_COLORS = ['#1f77b4', '#d62728', '#2ca02c', '#9467bd',
                  '#ff7f0e', '#17becf', '#8c564b', '#e377c2']


def _read_csv(csv_path):
    rows = []
    with open(csv_path, 'r') as f:
        reader = csv.DictReader(f)
        for row in reader:
            rows.append(row)
    return rows


def _filter_by_time(rows, t_range):
    """Tiene solo le righe con t_rel_sec dentro l'intervallo (t_min, t_max).
    t_range=None -> nessun filtro. Ogni estremo puo' essere None per lasciarlo aperto."""
    if not t_range:
        return rows
    t_min, t_max = t_range
    out = []
    for row in rows:
        try:
            t = float(row["t_rel_sec"])
        except (KeyError, ValueError):
            continue
        if t_min is not None and t < t_min:
            continue
        if t_max is not None and t > t_max:
            continue
        out.append(row)
    return out


def _parse_sensors(rows):
    s0, s1 = [], []
    for row in rows:
        try:
            vals = ast.literal_eval(row["sensor_values"])
            s0.append(float(vals[0]) if len(vals) > 0 else 0.0)
            s1.append(float(vals[1]) if len(vals) > 1 else 0.0)
        except Exception:
            s0.append(0.0)
            s1.append(0.0)
    return s0, s1


def _trial_name(csv_path):
    return os.path.splitext(os.path.basename(csv_path))[0]


def _range_suffix(t_range):
    """Suffisso per il nome file / titolo quando si plotta solo un intervallo."""
    if not t_range:
        return ""
    t_min, t_max = t_range
    lo = "start" if t_min is None else f"{t_min:g}"
    hi = "end"   if t_max is None else f"{t_max:g}"
    return f"_{lo}-{hi}s"


def _save_fig(fig, save_dir, name):
    if save_dir:
        os.makedirs(save_dir, exist_ok=True)
        path = os.path.join(save_dir, name + ".png")
        fig.savefig(path, dpi=150, bbox_inches='tight')
        print(f"Salvato: {path}")


def plot_motor(csv_path, save_dir=None, t_range=None):
    rows = _filter_by_time(_read_csv(csv_path), t_range)
    trial = _trial_name(csv_path)
    t        = [float(r["t_rel_sec"])       for r in rows]
    target   = [float(r["tail_target_rad"]) for r in rows]
    real     = [float(r["real_position_rad"]) for r in rows]
    center   = [float(r["center_rad"])      for r in rows]

    fig, ax = plt.subplots(figsize=(12, 5))
    ax.plot(t, target, color='red',   linewidth=1.5, linestyle='--', label='Target [rad]')
    ax.plot(t, real,   color='blue',  linewidth=1.5, label='Real position [rad]')
    ax.plot(t, center, color='green', linewidth=2.0, label='Oscillation centre [rad]')
    ax.axhline(0, color='black', linewidth=0.8)
    if t_range:
        ax.set_xlim(t_range[0], t_range[1])
    ax.set_xlabel("Time [s]", fontsize=LABEL_SIZE)
    ax.set_ylabel("Angle [rad]", fontsize=LABEL_SIZE)
    ax.set_title(f"Motor position — {trial}", fontsize=TITLE_SIZE, fontweight='bold')
    ax.legend(fontsize=LEGEND_SIZE, loc='upper right')
    ax.tick_params(labelsize=TICK_SIZE)
    ax.grid(True)
    plt.tight_layout()
    _save_fig(fig, save_dir, f"{trial}_motor_position{_range_suffix(t_range)}")
    plt.show()


def plot_bending(csv_path, save_dir=None, t_range=None):
    rows = _filter_by_time(_read_csv(csv_path), t_range)
    trial = _trial_name(csv_path)
    t        = [float(r["t_rel_sec"]) for r in rows]
    s0, s1   = _parse_sensors(rows)

    fig, ax = plt.subplots(figsize=(12, 5))
    ax.plot(t, s0, color='darkorange', linewidth=1.2, label='Sensor 0 (bending)')
    ax.plot(t, s1, color='purple',     linewidth=1.2, label='Sensor 1 (bending)')
    if t_range:
        ax.set_xlim(t_range[0], t_range[1])
    ax.set_xlabel("Time [s]", fontsize=LABEL_SIZE)
    ax.set_ylabel("ADC value", fontsize=LABEL_SIZE)
    ax.set_title(f"Bending sensors — {trial}", fontsize=TITLE_SIZE, fontweight='bold')
    ax.legend(fontsize=LEGEND_SIZE, loc='upper right')
    ax.tick_params(labelsize=TICK_SIZE)
    ax.grid(True)
    plt.tight_layout()
    _save_fig(fig, save_dir, f"{trial}_bending_sensors{_range_suffix(t_range)}")
    plt.show()


def plot_combined_sweep(csv_path, save_dir=None, t_range=None):
    rows = _filter_by_time(_read_csv(csv_path), t_range)
    trial = _trial_name(csv_path)
    t    = [float(r["t_rel_sec"])    for r in rows]
    amp  = [float(r["tail_amp_rad"]) for r in rows]
    freq = [float(r["tail_freq_hz"]) for r in rows]

    color_amp  = '#1f77b4'
    color_freq = '#d62728'

    fig, ax = plt.subplots(figsize=(12, 5))
    ax.plot(t, amp, color=color_amp, linewidth=1.8, label='Amplitude [rad]')
    if t_range:
        ax.set_xlim(t_range[0], t_range[1])
    ax.set_xlabel("Time [s]", fontsize=LABEL_SIZE)
    ax.set_ylabel("Amplitude [rad]", fontsize=LABEL_SIZE, color=color_amp)
    ax.tick_params(axis='y', labelcolor=color_amp, labelsize=TICK_SIZE)
    ax.tick_params(axis='x', labelsize=TICK_SIZE)
    ax.grid(True, alpha=0.4)

    ax_freq = ax.twinx()
    ax_freq.plot(t, freq, color=color_freq, linewidth=1.8,
                 linestyle='--', label='Frequency [Hz]')
    ax_freq.set_ylabel("Frequency [Hz]", fontsize=LABEL_SIZE, color=color_freq)
    ax_freq.tick_params(axis='y', labelcolor=color_freq, labelsize=TICK_SIZE)

    lines  = ax.get_lines() + ax_freq.get_lines()
    labels = [l.get_label() for l in lines]
    ax.legend(lines, labels, fontsize=LEGEND_SIZE, loc='upper right')
    ax.set_title(f"Amplitude & frequency vs time — {trial}", fontsize=TITLE_SIZE, fontweight='bold')

    plt.tight_layout()
    _save_fig(fig, save_dir, f"{trial}_combined_sweep{_range_suffix(t_range)}")
    plt.show()


def plot_current(csv_path, save_dir=None, t_range=None):
    rows = _filter_by_time(_read_csv(csv_path), t_range)
    trial = _trial_name(csv_path)
    t       = [float(r["t_rel_sec"])         for r in rows]
    current = [float(r["present_current_ma"]) for r in rows]

    fig, ax = plt.subplots(figsize=(12, 5))
    ax.plot(t, current, color='green', linewidth=1.2, label='Motor current [mA]')
    ax.axhline(0, color='black', linewidth=0.8)
    if t_range:
        ax.set_xlim(t_range[0], t_range[1])
    ax.set_xlabel("Time [s]", fontsize=LABEL_SIZE)
    ax.set_ylabel("Current [mA]", fontsize=LABEL_SIZE)
    ax.set_title(f"Motor current — {trial}", fontsize=TITLE_SIZE, fontweight='bold')
    ax.legend(fontsize=LEGEND_SIZE, loc='upper right')
    ax.tick_params(labelsize=TICK_SIZE)
    ax.grid(True)
    plt.tight_layout()
    _save_fig(fig, save_dir, f"{trial}_motor_current{_range_suffix(t_range)}")
    plt.show()


# ======================================================================
#  CONFRONTO fra piu' CSV sullo stesso grafico
# ======================================================================

def _as_list(csv_paths):
    """Accetta sia una singola stringa che una lista di percorsi."""
    if isinstance(csv_paths, (str, bytes)):
        return [csv_paths]
    return list(csv_paths)


def _compare_suffix(csv_paths):
    """Nome file: primo_vs_secondo(_vs_...)."""
    return "_vs_".join(_trial_name(p) for p in csv_paths)


def compare_motor(csv_paths, save_dir=None, t_range=None, which="real"):
    """Sovrappone la posizione del motore di piu' CSV sullo stesso grafico.

    which:
        "real"   -> Real position [rad]   (default)
        "target" -> Target [rad]
        "center" -> Oscillation centre [rad]
    """
    paths = _as_list(csv_paths)
    col_map = {
        "real":   ("real_position_rad", "Real position"),
        "target": ("tail_target_rad",   "Target"),
        "center": ("center_rad",        "Oscillation centre"),
    }
    if which not in col_map:
        raise ValueError(f"which deve essere uno di {list(col_map)}")
    col, desc = col_map[which]

    fig, ax = plt.subplots(figsize=(12, 5))
    colors = cycle(COMPARE_COLORS)
    for path, color in zip(paths, colors):
        rows = _filter_by_time(_read_csv(path), t_range)
        t = [float(r["t_rel_sec"]) for r in rows]
        y = [float(r[col])         for r in rows]
        ax.plot(t, y, color=color, linewidth=1.5, label=_trial_name(path))

    ax.axhline(0, color='black', linewidth=0.8)
    if t_range:
        ax.set_xlim(t_range[0], t_range[1])
    ax.set_xlabel("Time [s]", fontsize=LABEL_SIZE)
    ax.set_ylabel("Angle [rad]", fontsize=LABEL_SIZE)
    ax.set_title(f"Motor position ({desc}) — confronto", fontsize=TITLE_SIZE, fontweight='bold')
    ax.legend(fontsize=LEGEND_SIZE, loc='upper right')
    ax.tick_params(labelsize=TICK_SIZE)
    ax.grid(True)
    plt.tight_layout()
    _save_fig(fig, save_dir, f"compare_motor_{which}_{_compare_suffix(paths)}{_range_suffix(t_range)}")
    plt.show()


def compare_current(csv_paths, save_dir=None, t_range=None):
    """Sovrappone la corrente del motore di piu' CSV sullo stesso grafico."""
    paths = _as_list(csv_paths)

    fig, ax = plt.subplots(figsize=(12, 5))
    colors = cycle(COMPARE_COLORS)
    for path, color in zip(paths, colors):
        rows = _filter_by_time(_read_csv(path), t_range)
        t       = [float(r["t_rel_sec"])          for r in rows]
        current = [float(r["present_current_ma"]) for r in rows]
        ax.plot(t, current, color=color, linewidth=1.2, label=_trial_name(path))

    ax.axhline(0, color='black', linewidth=0.8)
    if t_range:
        ax.set_xlim(t_range[0], t_range[1])
    ax.set_xlabel("Time [s]", fontsize=LABEL_SIZE)
    ax.set_ylabel("Current [mA]", fontsize=LABEL_SIZE)
    ax.set_title("Motor current — confronto", fontsize=TITLE_SIZE, fontweight='bold')
    ax.legend(fontsize=LEGEND_SIZE, loc='upper right')
    ax.tick_params(labelsize=TICK_SIZE)
    ax.grid(True)
    plt.tight_layout()
    _save_fig(fig, save_dir, f"compare_current_{_compare_suffix(paths)}{_range_suffix(t_range)}")
    plt.show()


def compare_bending(csv_paths, save_dir=None, t_range=None, sensor=0):
    """Sovrappone un sensore di bending (0 o 1) di piu' CSV sullo stesso grafico."""
    paths = _as_list(csv_paths)
    if sensor not in (0, 1):
        raise ValueError("sensor deve essere 0 o 1")

    fig, ax = plt.subplots(figsize=(12, 5))
    colors = cycle(COMPARE_COLORS)
    for path, color in zip(paths, colors):
        rows = _filter_by_time(_read_csv(path), t_range)
        t = [float(r["t_rel_sec"]) for r in rows]
        s0, s1 = _parse_sensors(rows)
        y = s0 if sensor == 0 else s1
        ax.plot(t, y, color=color, linewidth=1.2, label=_trial_name(path))

    if t_range:
        ax.set_xlim(t_range[0], t_range[1])
    ax.set_xlabel("Time [s]", fontsize=LABEL_SIZE)
    ax.set_ylabel("ADC value", fontsize=LABEL_SIZE)
    ax.set_title(f"Bending sensor {sensor} — confronto", fontsize=TITLE_SIZE, fontweight='bold')
    ax.legend(fontsize=LEGEND_SIZE, loc='upper right')
    ax.tick_params(labelsize=TICK_SIZE)
    ax.grid(True)
    plt.tight_layout()
    _save_fig(fig, save_dir, f"compare_bending_s{sensor}_{_compare_suffix(paths)}{_range_suffix(t_range)}")
    plt.show()


# ======================================================================
#  Configurazione
# ======================================================================

# CSV = 'logs/trial_20260922_135914.csv'

SAVE_DIR = 'plots'

# Per il confronto: elenca i due (o piu') CSV da sovrapporre.
CSV_LIST = [
    'logs/std_con_pinna.csv',
    'logs/std_senza_pinna.csv',
]

# Intervallo temporale da plottare, in secondi (colonna t_rel_sec).
# None  -> plotta tutto
# (5, 20)     -> solo da 5 a 20 s
# (10, None)  -> da 10 s fino alla fine
# (None, 30)  -> dall'inizio fino a 30 s
T_RANGE = (10,20)

# --- Grafici singolo trial ---
# plot_motor(CSV, SAVE_DIR, t_range=T_RANGE)
# plot_current(CSV, SAVE_DIR, t_range=T_RANGE)

# plot_bending(CSV, SAVE_DIR, t_range=T_RANGE)
# plot_combined_sweep(CSV, SAVE_DIR, t_range=T_RANGE)   # usare con trial combined_sweep

# --- Grafici di confronto fra due (o piu') trial ---
compare_motor(CSV_LIST, SAVE_DIR, t_range=T_RANGE, which="real")
compare_current(CSV_LIST, SAVE_DIR, t_range=T_RANGE)
# compare_bending(CSV_LIST, SAVE_DIR, t_range=T_RANGE, sensor=0)