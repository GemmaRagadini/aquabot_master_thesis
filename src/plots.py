import csv
import matplotlib.pyplot as plt

def plot_tail_amplitude(csv_path):
    t = []
    amp = []

    with open(csv_path, 'r') as f:
        reader = csv.DictReader(f)

        for row in reader:
            t.append(float(row["t_rel_sec"]))
            amp.append(float(row["tail_amp_rad"]))

    plt.figure(figsize=(10,5))
    plt.plot(t, amp)
    plt.xlabel("Tempo [s]")
    plt.ylabel("Ampiezza coda [rad]")
    plt.title("Ampiezza della coda nel tempo")
    plt.grid(True)
    plt.show()

plot_tail_amplitude('logs/trial_20260313_164615.csv')