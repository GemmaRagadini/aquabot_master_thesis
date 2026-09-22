#!/usr/bin/env python3
"""
preflight_random.py — verifica la modalita' random_continuous SENZA muovere il robot.

Due usi:

  1) SIMULAZIONE OFFLINE (default): riproduce fedelmente il generatore del nodo
     (stessa logica di master_node.py), controlla che il comando theta resti
     SEMPRE dentro i limiti meccanici e salva un grafico del moto.
         python3 preflight_random.py
         python3 preflight_random.py --minutes 10 --hold 4 --step-prob 0.5

  2) CONTROLLO DI UN CSV REALE gia' registrato (es. da un run a motore spento):
         python3 preflight_random.py --csv logs/trial_YYYYMMDD_HHMMSS.csv

In entrambi i casi stampa PASS/FAIL sui limiti e (per la simulazione) la
copertura dello spazio dei parametri nel tempo. Richiede solo matplotlib.
"""
import argparse, ast, csv, math, random
from collections import Counter

# --- limiti fisici del nodo (devono combaciare con master_node.py) ---
MAX_AMP = 0.519
BIAS    = 0.903
TMIN    = 0.385
TMAX    = 1.422
CTRL_DT = 0.1   # 1/control_rate_hz (10 Hz)


def clamp(x, lo, hi):
    return hi if x > hi else lo if x < lo else x


# ============================================================
#  Replica FEDELE del generatore random_continuous del nodo
# ============================================================
def _resolve(tr, p):
    if tr == 'mixed':
        return 'step' if random.random() < p else 'ramp'
    return tr


def _draw(tr, lo, hi, prev):
    if hi <= lo:
        return lo
    if tr == 'walk':
        return clamp(prev + random.uniform(-0.5, 0.5) * (hi - lo), lo, hi)
    return random.uniform(lo, hi)


def _seg(tr, s, t, al):
    return t if tr == 'step' else s + (t - s) * al


def simulate(minutes, hold_s, p_step, amp_rng, freq_rng, center_max, seed):
    random.seed(seed)
    amp_min, amp_max = amp_rng
    freq_min, freq_max = freq_rng
    a_s = a_t = 0.4; f_s = f_t = 0.5; c_s = c_t = 0.0
    ae = fe = ce = 'ramp'
    cur_a, cur_f, cur_c = 0.4, 0.5, BIAS
    seg = 0.0; init = False
    phase = 0.0
    T, A, F, C, TH = [], [], [], [], []
    steps = int(minutes * 60 / CTRL_DT)
    for i in range(steps):
        t = i * CTRL_DT
        newseg = False
        if not init:
            a_s, f_s, c_s = cur_a, cur_f, cur_c - BIAS
            a_t, f_t, c_t = a_s, f_s, c_s
            init = True; newseg = True
        elif t - seg >= hold_s:
            a_s, f_s, c_s = cur_a, cur_f, cur_c - BIAS
            newseg = True
        if newseg:
            seg = t
            ae, fe, ce = _resolve('mixed', p_step), _resolve('mixed', p_step), _resolve('mixed', p_step)
            a_t = clamp(_draw(ae, amp_min, amp_max, a_t), 0.0, MAX_AMP)
            f_t = _draw(fe, freq_min, freq_max, f_t)
            m = max(0.0, MAX_AMP - a_t)
            cm = min(max(0.0, center_max), m)
            c_t = _draw(ce, -cm, cm, c_t)
        al = clamp((t - seg) / hold_s, 0.0, 1.0)
        cur_a = clamp(_seg(ae, a_s, a_t, al), 0.0, MAX_AMP)
        cur_f = _seg(fe, f_s, f_t, al)
        coff = _seg(ce, c_s, c_t, al)
        cap = max(0.0, MAX_AMP - cur_a)
        coff = clamp(coff, -cap, cap)
        cur_c = BIAS + coff
        # fase continua come nel nodo (_advance_phase) e posizione comandata
        import math
        phase += 2.0 * math.pi * cur_f * CTRL_DT
        theta = clamp(cur_c + cur_a * math.sin(phase), TMIN, TMAX)
        T.append(t); A.append(cur_a); F.append(cur_f); C.append(cur_c); TH.append(theta)
    return T, A, F, C, TH


def coverage_report(minutes, hold_s, p_step, amp_rng, freq_rng, center_max, seed):
    amp_min, amp_max = amp_rng
    freq_min, freq_max = freq_rng
    NA, NF, NC = 6, 6, 5

    def cell(a, f, coff):
        ia = int((a - amp_min) / (amp_max - amp_min) * NA) if amp_max > amp_min else 0
        jf = int((f - freq_min) / (freq_max - freq_min) * NF) if freq_max > freq_min else 0
        kc = int((coff + center_max) / (2 * center_max) * NC) if center_max > 0 else 0
        return (min(NA - 1, max(0, ia)), min(NF - 1, max(0, jf)), min(NC - 1, max(0, kc)))

    # celle raggiungibili (run lungo)
    random.seed(seed + 999)
    big = _sweep_cells(600, hold_s, p_step, amp_rng, freq_rng, center_max, cell)
    reach = len(big)

    random.seed(seed)
    hits = Counter()
    amp_min, amp_max = amp_rng
    a_s = a_t = 0.4; f_s = f_t = 0.5; c_s = c_t = 0.0
    ae = fe = ce = 'ramp'; cur_a, cur_f, cur_c = 0.4, 0.5, BIAS
    seg = 0.0; init = False
    steps = int(minutes * 60 / CTRL_DT)
    marks = [5, 10, 15, 20, 30, 45, 60]
    rows = []
    next_mark = 0
    for i in range(steps):
        t = i * CTRL_DT; newseg = False
        if not init:
            a_s, f_s, c_s = cur_a, cur_f, cur_c - BIAS
            a_t, f_t, c_t = a_s, f_s, c_s; init = True; newseg = True
        elif t - seg >= hold_s:
            a_s, f_s, c_s = cur_a, cur_f, cur_c - BIAS; newseg = True
        if newseg:
            seg = t
            ae, fe, ce = _resolve('mixed', p_step), _resolve('mixed', p_step), _resolve('mixed', p_step)
            a_t = clamp(_draw(ae, amp_min, amp_max, a_t), 0.0, MAX_AMP)
            f_t = _draw(fe, freq_min, freq_max, f_t)
            m = max(0.0, MAX_AMP - a_t); cm = min(max(0.0, center_max), m)
            c_t = _draw(ce, -cm, cm, c_t)
        al = clamp((t - seg) / hold_s, 0.0, 1.0)
        cur_a = clamp(_seg(ae, a_s, a_t, al), 0.0, MAX_AMP)
        cur_f = _seg(fe, f_s, f_t, al)
        coff = _seg(ce, c_s, c_t, al); cap = max(0.0, MAX_AMP - cur_a)
        coff = clamp(coff, -cap, cap)
        hits[cell(cur_a, cur_f, coff)] += 1
        mm = t / 60.0
        if next_mark < len(marks) and mm >= marks[next_mark]:
            v1 = sum(1 for v in hits.values() if v >= 1)
            v5 = sum(1 for v in hits.values() if v >= 5)
            v10 = sum(1 for v in hits.values() if v >= 10)
            rows.append((marks[next_mark], v1, v5, v10))
            next_mark += 1
    return reach, rows


def _sweep_cells(minutes, hold_s, p_step, amp_rng, freq_rng, center_max, cell):
    amp_min, amp_max = amp_rng; freq_min, freq_max = freq_rng
    a_s = a_t = 0.4; f_s = f_t = 0.5; c_s = c_t = 0.0
    ae = fe = ce = 'ramp'; cur_a, cur_f, cur_c = 0.4, 0.5, BIAS
    seg = 0.0; init = False; seen = set()
    steps = int(minutes * 60 / CTRL_DT)
    for i in range(steps):
        t = i * CTRL_DT; newseg = False
        if not init:
            a_s, f_s, c_s = cur_a, cur_f, cur_c - BIAS
            a_t, f_t, c_t = a_s, f_s, c_s; init = True; newseg = True
        elif t - seg >= hold_s:
            a_s, f_s, c_s = cur_a, cur_f, cur_c - BIAS; newseg = True
        if newseg:
            seg = t
            ae, fe, ce = _resolve('mixed', p_step), _resolve('mixed', p_step), _resolve('mixed', p_step)
            a_t = clamp(_draw(ae, amp_min, amp_max, a_t), 0.0, MAX_AMP)
            f_t = _draw(fe, freq_min, freq_max, f_t)
            m = max(0.0, MAX_AMP - a_t); cm = min(max(0.0, center_max), m)
            c_t = _draw(ce, -cm, cm, c_t)
        al = clamp((t - seg) / hold_s, 0.0, 1.0)
        cur_a = clamp(_seg(ae, a_s, a_t, al), 0.0, MAX_AMP)
        cur_f = _seg(fe, f_s, f_t, al)
        coff = _seg(ce, c_s, c_t, al); cap = max(0.0, MAX_AMP - cur_a)
        coff = clamp(coff, -cap, cap)
        seen.add(cell(cur_a, cur_f, coff))
    return seen


# ============================================================
#  Controllo di un CSV reale
# ============================================================
def check_csv(path):
    t, target, center, amp, freq = [], [], [], [], []
    with open(path) as f:
        r = csv.DictReader(f)
        for row in r:
            try:
                t.append(float(row['t_rel_sec']))
                target.append(float(row['tail_target_rad']))
                center.append(float(row['center_rad']))
                amp.append(float(row['tail_amp_rad']))
                freq.append(float(row['tail_freq_hz']))
            except (KeyError, ValueError):
                continue
    if not target:
        print("Nessuna riga valida nel CSV."); return None
    tmn, tmx = min(target), max(target)
    ok = tmn >= TMIN - 1e-3 and tmx <= TMAX + 1e-3
    print(f"CSV: {path}")
    print(f"  campioni: {len(target)}   durata: {t[-1]-t[0]:.1f}s")
    print(f"  theta (comando) in [{tmn:.4f}, {tmx:.4f}]  limiti [{TMIN}, {TMAX}]")
    print(f"  amp in [{min(amp):.3f}, {max(amp):.3f}]  freq in [{min(freq):.3f}, {max(freq):.3f}]  "
          f"center in [{min(center):.3f}, {max(center):.3f}]")
    print("  RISULTATO:", "PASS — comando sempre nei limiti" if ok else "FAIL — comando fuori dai limiti!")
    return t, target, center, amp, freq


# ============================================================
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--csv', help='controlla un CSV reale invece di simulare')
    ap.add_argument('--minutes', type=float, default=60.0)
    ap.add_argument('--hold', type=float, default=4.0)
    ap.add_argument('--step-prob', type=float, default=0.5)
    ap.add_argument('--amp-min', type=float, default=0.30)
    ap.add_argument('--amp-max', type=float, default=0.518)
    ap.add_argument('--freq-min', type=float, default=0.5)
    ap.add_argument('--freq-max', type=float, default=1.0)
    ap.add_argument('--center-max', type=float, default=0.20)
    ap.add_argument('--seed', type=int, default=42)
    ap.add_argument('--window', type=float, default=120.0,
                    help='secondi da plottare (zoom sui salti). Default 120.')
    ap.add_argument('--out', default='preflight_random.png')
    args = ap.parse_args()

    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt

    amp_rng = (args.amp_min, args.amp_max)
    freq_rng = (args.freq_min, args.freq_max)

    if args.csv:
        res = check_csv(args.csv)
        if res is None:
            return
        t, target, center, amp, freq = res
        fig, ax = plt.subplots(2, 1, figsize=(12, 6), sharex=True)
        ax[0].plot(t, target, lw=0.6, label='theta comando')
        ax[0].plot(t, center, lw=0.8, label='center')
        ax[0].axhline(TMIN, color='r', ls='--', lw=0.8); ax[0].axhline(TMAX, color='r', ls='--', lw=0.8)
        ax[0].set_ylabel('rad'); ax[0].legend(loc='upper right'); ax[0].set_title(f'CSV reale: {args.csv}')
        ax[1].plot(t, amp, label='amp'); ax[1].plot(t, freq, label='freq')
        ax[1].set_xlabel('t_rel [s]'); ax[1].legend(loc='upper right')
        fig.tight_layout(); fig.savefig(args.out, dpi=110)
        print(f"Grafico salvato in {args.out}")
        return

    # --- simulazione ---
    print("=== SIMULAZIONE OFFLINE (nessun robot) ===")
    print(f"minutes={args.minutes} hold={args.hold}s step_prob={args.step_prob} "
          f"amp={amp_rng} freq={freq_rng} center_max={args.center_max}\n")

    # sicurezza: theta = center + amp*sin(phase); estremo = center +/- amp
    T, A, F, C, TH = simulate(args.minutes, args.hold, args.step_prob, amp_rng, freq_rng, args.center_max, args.seed)
    hi = max(c + a for c, a in zip(C, A))
    lo = min(c - a for c, a in zip(C, A))
    ok = hi <= TMAX + 1e-9 and lo >= TMIN - 2e-3
    print(f"Estremi del comando theta = center +/- amp: [{lo:.4f}, {tmax_str(hi)}]  limiti [{TMIN}, {TMAX}]")
    print("RISULTATO SICUREZZA:", "PASS — mai fuori dai limiti" if ok else "FAIL!")
    print()

    reach, rows = coverage_report(min(args.minutes, 60), args.hold, args.step_prob,
                                  amp_rng, freq_rng, args.center_max, args.seed)
    print(f"Copertura spazio (amp x freq x center), {reach} celle raggiungibili:")
    print(f"  {'min':>4} | {'>=1':>9} | {'>=5':>9} | {'>=10 volte':>10}")
    for k, v1, v5, v10 in rows:
        print(f"  {k:>4} | {v1:>3}/{reach} {100*v1/reach:3.0f}% | {v5:>3}/{reach} {100*v5/reach:3.0f}% | "
              f"{v10:>3}/{reach} {100*v10/reach:3.0f}%")

    # grafico: finestra scelta (default 120 s) per vedere bene i salti
    n = min(len(T), int(args.window / CTRL_DT))
    fig, ax = plt.subplots(2, 1, figsize=(12, 6), sharex=True)
    ax[0].plot(T[:n], TH[:n], lw=0.8, color='C0', label='posizione comandata (theta)')
    ax[0].plot(T[:n], C[:n], lw=1.4, color='C3', label='center')
    ax[0].plot(T[:n], [c + a for c, a in zip(C[:n], A[:n])], lw=0.6, color='0.6', label='inviluppo +/- amp')
    ax[0].plot(T[:n], [c - a for c, a in zip(C[:n], A[:n])], lw=0.6, color='0.6')
    ax[0].axhline(TMIN, color='r', ls='--', lw=0.8); ax[0].axhline(TMAX, color='r', ls='--', lw=0.8)
    ax[0].set_ylabel('rad'); ax[0].legend(loc='upper right', fontsize=8)
    ax[0].set_title(f'random_continuous — simulazione (primi {int(args.window)} s). Cerca i cambi NETTI (gradini) e CONTINUI (rampe).')
    ax[1].step(T[:n], A[:n], where='post', label='amp')
    ax[1].step(T[:n], F[:n], where='post', label='freq')
    ax[1].set_xlabel('t [s]'); ax[1].legend(loc='upper right')
    fig.tight_layout(); fig.savefig(args.out, dpi=110)
    print(f"\nGrafico salvato in {args.out}")


def tmax_str(x):
    return f"{x:.4f}"


if __name__ == '__main__':
    main()