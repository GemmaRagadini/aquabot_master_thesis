import ast
import csv
import os
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches

LABEL_SIZE  = 14
TITLE_SIZE  = 16
LEGEND_SIZE = 12
TICK_SIZE   = 12


def _read_csv(csv_path):
    rows = []
    with open(csv_path, 'r') as f:
        reader = csv.DictReader(f)
        for row in reader:
            rows.append(row)
    return rows


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


def _save_fig(fig, save_dir, name):
    if save_dir:
        os.makedirs(save_dir, exist_ok=True)
        path = os.path.join(save_dir, name + ".png")
        fig.savefig(path, dpi=150, bbox_inches='tight')
        print(f"Salvato: {path}")


def plot_motor(csv_path, save_dir=None):
    rows = _read_csv(csv_path)
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
    ax.set_xlabel("Time [s]", fontsize=LABEL_SIZE)
    ax.set_ylabel("Angle [rad]", fontsize=LABEL_SIZE)
    ax.set_title(f"Motor position — {trial}", fontsize=TITLE_SIZE, fontweight='bold')
    ax.legend(fontsize=LEGEND_SIZE, loc='upper right')
    ax.tick_params(labelsize=TICK_SIZE)
    ax.grid(True)
    plt.tight_layout()
    _save_fig(fig, save_dir, f"{trial}_motor_position")
    plt.show()


def plot_bending(csv_path, save_dir=None):
    rows = _read_csv(csv_path)
    trial = _trial_name(csv_path)
    t        = [float(r["t_rel_sec"]) for r in rows]
    s0, s1   = _parse_sensors(rows)

    fig, ax = plt.subplots(figsize=(12, 5))
    ax.plot(t, s0, color='darkorange', linewidth=1.2, label='Sensor 0 (bending)')
    ax.plot(t, s1, color='purple',     linewidth=1.2, label='Sensor 1 (bending)')
    ax.set_xlabel("Time [s]", fontsize=LABEL_SIZE)
    ax.set_ylabel("ADC value", fontsize=LABEL_SIZE)
    ax.set_title(f"Bending sensors — {trial}", fontsize=TITLE_SIZE, fontweight='bold')
    ax.legend(fontsize=LEGEND_SIZE, loc='upper right')
    ax.tick_params(labelsize=TICK_SIZE)
    ax.grid(True)
    plt.tight_layout()
    _save_fig(fig, save_dir, f"{trial}_bending_sensors")
    plt.show()


def plot_combined_sweep(csv_path, save_dir=None):
    rows = _read_csv(csv_path)
    trial = _trial_name(csv_path)
    t    = [float(r["t_rel_sec"])    for r in rows]
    amp  = [float(r["tail_amp_rad"]) for r in rows]
    freq = [float(r["tail_freq_hz"]) for r in rows]

    color_amp  = '#1f77b4'
    color_freq = '#d62728'

    fig, ax = plt.subplots(figsize=(12, 5))
    ax.plot(t, amp, color=color_amp, linewidth=1.8, label='Amplitude [rad]')
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
    _save_fig(fig, save_dir, f"{trial}_combined_sweep")
    plt.show()


def plot_current(csv_path, save_dir=None):
    rows = _read_csv(csv_path)
    trial = _trial_name(csv_path)
    t       = [float(r["t_rel_sec"])         for r in rows]
    current = [float(r["present_current_ma"]) for r in rows]

    fig, ax = plt.subplots(figsize=(12, 5))
    ax.plot(t, current, color='green', linewidth=1.2, label='Motor current [mA]')
    ax.axhline(0, color='black', linewidth=0.8)
    ax.set_xlabel("Time [s]", fontsize=LABEL_SIZE)
    ax.set_ylabel("Current [mA]", fontsize=LABEL_SIZE)
    ax.set_title(f"Motor current — {trial}", fontsize=TITLE_SIZE, fontweight='bold')
    ax.legend(fontsize=LEGEND_SIZE, loc='upper right')
    ax.tick_params(labelsize=TICK_SIZE)
    ax.grid(True)
    plt.tight_layout()
    _save_fig(fig, save_dir, f"{trial}_motor_current")
    plt.show()


CSV = 'logs/trial_20260910_150350.csv'
SAVE_DIR = 'plots'

plot_motor(CSV, SAVE_DIR)
plot_current(CSV, SAVE_DIR)

plot_bending(CSV, SAVE_DIR)
# plot_combined_sweep(CSV, SAVE_DIR)   # usare con trial combined_sweep