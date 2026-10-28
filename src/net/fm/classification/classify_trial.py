"""
Runs the existing detector on one or more trials and classifies every event
by physical cause (classifier.py). The detector files are only IMPORTED, never
modified.

For every trial:
  replay through the detector -> episodes (fast, slow, start-of-session check)
  -> signature -> physical rules -> signature library
Prints one line per episode and saves everything to a JSON file.

Usage (from the aquabot folder):
  python3 src/net/fm/classification/classify_trial.py \
      --checkpoint src/net/fm/checkpoints_fm/best_fm_v2.pt \
      --scaler_path src/net/scaler/scalers_joint_v2.pkl \
      --calib src/net/fm/checkpoints_fm/detector_v2.json \
      --dataset_dir src/net/test_tocchi --trial trial_TOUCH_1.csv

  all the trials of a folder, building/updating a library:
      ... --dataset_dir src/net/test_set_v2 --all_trials \
          --library src/net/fm/classification/library.json --update_library

  software sensor fault on nominal data (validation), repeatable:
      ... --fault sensor_diff:stuck:12:0  --fault current:offset:15:40
      channel:type:t_start_s:value   type = stuck | offset | drift | noise | zero
      value in physical units (sensor units, mA; drift per second; noise std)

  suggested threshold "new vs already seen" from repeated perturbations
  (folder where the trial names start with the perturbation name, e.g.
  weight20g_1.csv, weight20g_2.csv, contact_1.csv ...):
      ... --dataset_dir <folder> --all_trials --suggest_tau
"""
import argparse
import json
import os
import re
import sys

import numpy as np
import torch

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
DETECTION_DIR = os.path.join(SCRIPT_DIR, "..", "detection")
sys.path.insert(0, SCRIPT_DIR)
sys.path.insert(0, os.path.abspath(DETECTION_DIR))

from det_common import (DEFAULT_DATASET, DEFAULT_SCALER, default_calib_path,  # noqa: E402
                        load_dataset, load_model, replay, resolve_trial, trial_series)
from detector import Calibration, PerturbationDetector                         # noqa: E402
from classifier import (ClassifierConfig, SignatureLibrary, apply_rules,      # noqa: E402
                        compute_signature, episodes_from_replay, inject_sensor_fault,
                        startup_episode, FAULT_TYPES, CHANNELS)


def parse_fault(spec):
    try:
        ch, kind, t0, val = spec.split(":")
        t0, val = float(t0), float(val)
    except ValueError:
        raise SystemExit(f"--fault '{spec}': format channel:type:t_start_s:value")
    if ch not in CHANNELS or kind not in FAULT_TYPES:
        raise SystemExit(f"--fault '{spec}': channel in {CHANNELS}, type in {FAULT_TYPES}")
    return ch, kind, t0, val


def to_jsonable(x):
    if isinstance(x, dict):
        return {k: to_jsonable(v) for k, v in x.items()}
    if isinstance(x, (list, tuple)):
        return [to_jsonable(v) for v in x]
    if isinstance(x, (np.floating, np.integer)):
        return x.item()
    if isinstance(x, np.ndarray):
        return x.tolist()
    if isinstance(x, float) and not np.isfinite(x):
        return None
    return x


def fmt_episode(sig, rule, lib_res):
    t_end = sig.get("t_end_s")
    span = f"{sig['t_onset_s']:6.1f}-{t_end:6.1f} s" if t_end is not None else \
           f"{sig['t_onset_s']:6.1f}- (open)  "
    lab = rule["label"] if rule["status"] == "classified" else rule["status"].upper()
    if rule["subtype"]:
        lab += f" [{rule['subtype']}]"
    s = f"  {span}  {sig['kind']:10s} -> {lab:32s} conf {rule['confidence']:.2f}"
    if lib_res["status"] != "skipped":
        d = "" if lib_res["distance"] is None else f" d={lib_res['distance']:.2f}"
        s += f" | library: {lib_res['status']} #{lib_res['entry_id']} {lib_res['entry_name']}{d}"
        if lib_res["doubtful"]:
            s += "  ** DOUBTFUL **"
    return s


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--checkpoint", required=True)
    ap.add_argument("--calib", default=None, help="JSON from calibrate.py")
    ap.add_argument("--dataset_dir", default=DEFAULT_DATASET)
    ap.add_argument("--scaler_path", default=DEFAULT_SCALER)
    ap.add_argument("--device", default="cpu")
    ap.add_argument("--trial", action="append", default=[],
                    help="trial name (repeatable). Default: first validation trial")
    ap.add_argument("--all_trials", action="store_true", help="all the trials of --dataset_dir")
    ap.add_argument("--fault", action="append", default=[],
                    help="software sensor fault channel:type:t_start_s:value (repeatable)")
    ap.add_argument("--library", default=None, help="library JSON (created if missing)")
    ap.add_argument("--update_library", action="store_true",
                    help="add/update the library entries and save it")
    ap.add_argument("--no_startup_check", action="store_true")
    ap.add_argument("--suggest_tau", action="store_true",
                    help="threshold new/known from repeated perturbations (trial name prefix = group)")
    ap.add_argument("--tau", type=float, default=None, help="override tau_known")
    ap.add_argument("--out", default=os.path.join(SCRIPT_DIR, "classification_out", "episodes.json"))
    args = ap.parse_args()

    cfg = ClassifierConfig()
    if args.tau is not None:
        cfg.tau_known = args.tau

    device = torch.device(args.device)
    FM, ckpt = load_model(args.checkpoint, device)
    calib = Calibration.load(args.calib or default_calib_path(args.checkpoint))
    ds, val_trials = load_dataset(args.dataset_dir, args.scaler_path, ckpt)
    norm_stats = ckpt.get("norm_stats")
    faults = [parse_fault(f) for f in args.fault]

    if args.all_trials:
        trials = list(range(len(ds.trial_names)))
    elif args.trial:
        trials = [resolve_trial(ds, t, val_trials) for t in args.trial]
    else:
        trials = [resolve_trial(ds, None, val_trials)]

    lib = SignatureLibrary.load(args.library, cfg) if args.library else SignatureLibrary(cfg)
    results, groups = [], {}
    for ti in trials:
        name = ds.trial_names[ti]
        cmd, sens = trial_series(ds, ti)
        for k, (ch, kind, t0, val) in enumerate(faults):
            sens = inject_sensor_fault(sens, ch, kind, t0, val, hz=calib.config.hz,
                                       norm_stats=norm_stats,
                                       rng=np.random.default_rng(1000 + k))
        det = PerturbationDetector(FM, calib, H=ds.h, device=args.device, norm_stats=norm_stats)
        r = replay(det, cmd, sens)

        eps = episodes_from_replay(r, det, cmd, cfg)
        if not args.no_startup_check:
            st = startup_episode(r, det, cmd, cfg)
            if st is not None:
                eps.insert(0, st)

        print(f"\n{name} | {len(cmd) / calib.config.hz:.1f} s | {len(det.events)} fast, "
              f"{len(det.slow_events)} slow events -> {len(eps)} episodes"
              + (f" | faults: {args.fault}" if faults else ""))
        if not eps:
            print("  no event")
        for ep in eps:
            sig = compute_signature(ep, r, cmd, sens, det, norm_stats, cfg)
            rule = apply_rules(sig, cfg)
            lib_res = lib.process(sig, rule, source=name, update=args.update_library)
            print(fmt_episode(sig, rule, lib_res))
            for reason in rule["reasons"]:
                print(f"        - {reason}")
            if lib_res["doubtful"]:
                print(f"        - doubtful: {lib_res['doubt_reason']}")
            results.append({"trial": name, "faults": args.fault, "signature": sig,
                            "rules": rule, "library": lib_res})
            if rule["status"] == "classified":
                groups.setdefault(re.sub(r"_?\d+\.csv$", "", name), []).append(sig)

    if args.library:
        print("\nLibrary:")
        print(lib.summary())
        if args.update_library:
            lib.save(args.library)
            print(f"Saved {args.library}")

    if args.suggest_tau:
        if len(groups) < 2:
            print("\n--suggest_tau needs at least 2 groups (trial name prefixes) with classified events")
        else:
            tau, table = SignatureLibrary.suggest_tau(groups, cfg)
            print("\nThreshold new / already seen (distance of the repetitions):")
            for row in table:
                print(f"  {row['group']:20s} n={row['n']:<3d} max within {row['within_max']:.2f}"
                      f" | min to other groups {row['between_min']:.2f}")
            print(f"  suggested tau_known = {tau:.2f} (current {cfg.tau_known})")

    os.makedirs(os.path.dirname(args.out) or ".", exist_ok=True)
    with open(args.out, "w") as f:
        json.dump(to_jsonable(results), f, indent=2)
    print(f"\nSaved {args.out} ({len(results)} episodes)")


if __name__ == "__main__":
    main()