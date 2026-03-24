import csv
import matplotlib.pyplot as plt

def plot_tail_amplitude(csv_path):
    t = []
    amp = []
    bias_total = []
    center_upper = []
    center_lower = []

    with open(csv_path, 'r') as f:
        reader = csv.DictReader(f)
        for row in reader:
            t_val = float(row["t_rel_sec"])
            amp_val = float(row["tail_amp_rad"])
            bias_val = float(row["tail_bias_rad"]) + float(row["tail_bias_offset_rad"])

            t.append(t_val)
            amp.append(amp_val)
            bias_total.append(bias_val)
            center_upper.append(bias_val + amp_val)
            center_lower.append(bias_val - amp_val)

    plt.figure(figsize=(12, 6))

    # banda dell'oscillazione
    plt.fill_between(t, center_lower, center_upper, alpha=0.2, color='blue', label='Banda oscillazione')

    # limiti superiore e inferiore
    plt.plot(t, center_upper, color='blue', linewidth=1, linestyle='--', label='bias + amp')
    plt.plot(t, center_lower, color='blue', linewidth=1, linestyle='--', label='bias - amp')

    # centro oscillazione totale (bias fisso + offset feedback)
    plt.plot(t, bias_total, color='red', linewidth=2, label='Centro oscillazione (bias + feedback)')

    # ampiezza istantanea
    plt.plot(t, amp, color='green', linewidth=1.5, linestyle=':', label='Ampiezza')

    # linea dello zero
    plt.axhline(0, color='black', linewidth=0.8, linestyle='-')

    plt.xlabel("Tempo [s]")
    plt.ylabel("Angolo [rad]")
    plt.title("Ampiezza e centro della coda nel tempo")
    plt.legend()
    plt.grid(True)
    plt.tight_layout()
    plt.show()

plot_tail_amplitude('logs/trial_20260318_151401.csv')