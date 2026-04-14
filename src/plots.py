import ast
import csv
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


def plot_motor(csv_path):
    rows = _read_csv(csv_path)
    t        = [float(r["t_rel_sec"])          for r in rows]
    target   = [float(r["tail_target_rad"])    for r in rows]
    position = [float(r["present_position_rad"]) for r in rows]

    fig, ax = plt.subplots(figsize=(12, 5))
    ax.plot(t, target,   color='red',  linewidth=1.5, linestyle='--', label='Target [rad]')
    ax.plot(t, position, color='blue', linewidth=1.5, label='Motor position [rad]')
    ax.axhline(0, color='black', linewidth=0.8)
    ax.set_xlabel("Time [s]", fontsize=LABEL_SIZE)
    ax.set_ylabel("Angle [rad]", fontsize=LABEL_SIZE)
    ax.set_title("Motor position", fontsize=TITLE_SIZE, fontweight='bold')
    ax.legend(fontsize=LEGEND_SIZE, loc='upper right')
    ax.tick_params(labelsize=TICK_SIZE)
    ax.grid(True)
    plt.tight_layout()
    plt.show()


def plot_sensors(csv_path):
    rows = _read_csv(csv_path)
    t        = [float(r["t_rel_sec"]) for r in rows]
    s0, s1   = _parse_sensors(rows)

    fig, ax = plt.subplots(figsize=(12, 5))
    ax.plot(t, s0, color='darkorange', linewidth=1.2, label='Sensor 0 (bending)')
    ax.plot(t, s1, color='purple',     linewidth=1.2, label='Sensor 1 (bending)')
    ax.set_xlabel("Time [s]", fontsize=LABEL_SIZE)
    ax.set_ylabel("ADC value", fontsize=LABEL_SIZE)
    ax.set_title("Bending sensors", fontsize=TITLE_SIZE, fontweight='bold')
    ax.legend(fontsize=LEGEND_SIZE, loc='upper right')
    ax.tick_params(labelsize=TICK_SIZE)
    ax.grid(True)
    plt.tight_layout()
    plt.show()


def plot_tail_amplitude(csv_path):
    rows = _read_csv(csv_path)
    t, amp, bias_total = [], [], []
    center_upper, center_lower = [], []

    for row in rows:
        t_val    = float(row["t_rel_sec"])
        amp_val  = float(row["tail_amp_rad"])
        bias_val = float(row["tail_bias_rad"]) + float(row["tail_bias_offset_rad"])
        t.append(t_val)
        amp.append(amp_val)
        bias_total.append(bias_val)
        center_upper.append(bias_val + amp_val)
        center_lower.append(bias_val - amp_val)

    fig, ax = plt.subplots(figsize=(12, 5))

    # banda oscillazione
    ax.fill_between(t, center_lower, center_upper,
                    alpha=0.15, color='blue', label='Oscillation band')
    ax.plot(t, center_upper, color='blue', linewidth=1,   linestyle='--', label='bias + amp')
    ax.plot(t, center_lower, color='blue', linewidth=1,   linestyle='--', label='bias − amp')

    # centro oscillazione — evidenziato
    ax.plot(t, bias_total, color='red', linewidth=2.5, zorder=5,
            label='Oscillation centre (bias + feedback)')
    ax.fill_between(t,
                    [b - 0.015 for b in bias_total],
                    [b + 0.015 for b in bias_total],
                    color='red', alpha=0.25, zorder=4)

    # ampiezza istantanea
    ax.plot(t, amp, color='green', linewidth=1.5, linestyle=':', label='Amplitude')
    ax.axhline(0, color='black', linewidth=0.8)

    ax.set_xlabel("Time [s]", fontsize=LABEL_SIZE)
    ax.set_ylabel("Angle [rad]", fontsize=LABEL_SIZE)
    ax.set_title("Tail amplitude and oscillation centre", fontsize=TITLE_SIZE, fontweight='bold')
    ax.legend(fontsize=LEGEND_SIZE, loc='upper right')
    ax.tick_params(labelsize=TICK_SIZE)
    ax.grid(True)
    plt.tight_layout()
    plt.show()


CSV = 'logs/trial_20260414_113250.csv'

plot_motor(CSV)
plot_tail_amplitude(CSV)
plot_sensors(CSV)