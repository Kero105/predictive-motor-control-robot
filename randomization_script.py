"""
Randomized Trial Order Generator
Generates the fixed, pre-registered trial sequence for data collection, per
experimental_protocol.md Section 10. Run this ONCE before data collection begins,
save the output CSV, and follow it exactly during testing (log any deviations).
"""

import random
import csv

# --- Set a fixed seed so this sequence is reproducible/verifiable, but only
#     generate it ONCE and then treat the output as locked. ---
RANDOM_SEED = 2026
random.seed(RANDOM_SEED)

CONTROLLERS = ["fixed_speed", "pid", "threshold", "predictive", "hybrid"]

CALIBRATION_TERRAINS = ["smooth", "carpet_A", "ramp_angle_1"]
PAYLOADS_G = [0, 200, 400]

HELD_OUT_TERRAINS = ["carpet_B", "ramp_angle_2"]
HELD_OUT_PAYLOAD_G = 200  # single representative payload level for generalization trials

EDGE_CASE_CONDITIONS = [
    {"terrain": "smooth", "payload_g": 200, "condition": "low_battery"},
    {"terrain": "smooth", "payload_g": 200, "condition": "unanticipated_obstacle"},
]

REPS_PER_CONDITION = 5  # Reduced from 8 for feasibility — update after running your actual
                          # power analysis (protocol Section 11); this is a realistic starting
                          # default, not a final number. At ~2 min/trial, this generates roughly
                          # 325 total trials (~11 hours of testing), spread over multiple sessions,
                          # instead of the original 520-trial (~17 hour) sequence.


def build_trial_list():
    trials = []
    trial_id = 1

    # --- Calibration set: full factorial, controller x terrain x payload ---
    for controller in CONTROLLERS:
        for terrain in CALIBRATION_TERRAINS:
            for payload in PAYLOADS_G:
                for rep in range(REPS_PER_CONDITION):
                    trials.append({
                        "trial_id": trial_id,
                        "controller": controller,
                        "terrain": terrain,
                        "payload_g": payload,
                        "condition_type": "calibration",
                        "repetition": rep + 1,
                    })
                    trial_id += 1

    # --- Held-out generalization set ---
    for controller in CONTROLLERS:
        for terrain in HELD_OUT_TERRAINS:
            for rep in range(REPS_PER_CONDITION):
                trials.append({
                    "trial_id": trial_id,
                    "controller": controller,
                    "terrain": terrain,
                    "payload_g": HELD_OUT_PAYLOAD_G,
                    "condition_type": "held_out_validation",
                    "repetition": rep + 1,
                })
                trial_id += 1

    # --- Edge-case / robustness set ---
    for controller in CONTROLLERS:
        for edge in EDGE_CASE_CONDITIONS:
            for rep in range(REPS_PER_CONDITION):
                trials.append({
                    "trial_id": trial_id,
                    "controller": controller,
                    "terrain": edge["terrain"],
                    "payload_g": edge["payload_g"],
                    "condition_type": edge["condition"],
                    "repetition": rep + 1,
                })
                trial_id += 1

    return trials


def randomize_order(trials):
    """Shuffles the RUN ORDER while keeping the original trial_id as a stable
    identifier — so 'trial_id' tells you what the trial IS, and 'run_order'
    tells you WHEN to run it."""
    shuffled = trials.copy()
    random.shuffle(shuffled)
    for i, trial in enumerate(shuffled):
        trial["run_order"] = i + 1
    return shuffled


def save_sequence(trials, filename="trial_sequence.csv"):
    fieldnames = ["run_order", "trial_id", "controller", "terrain", "payload_g",
                  "condition_type", "repetition", "completed", "notes"]
    with open(filename, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        for trial in sorted(trials, key=lambda t: t["run_order"]):
            row = {k: trial.get(k, "") for k in fieldnames}
            row["completed"] = ""  # fill in during testing
            row["notes"] = ""      # fill in during testing (deviations, failures, etc.)
            writer.writerow(row)
    print(f"Saved {len(trials)} trials to {filename}")


if __name__ == "__main__":
    trials = build_trial_list()
    print(f"Total trials generated: {len(trials)}")
    print(f"  Calibration trials: {sum(1 for t in trials if t['condition_type'] == 'calibration')}")
    print(f"  Held-out validation trials: {sum(1 for t in trials if t['condition_type'] == 'held_out_validation')}")
    print(f"  Edge-case trials: {sum(1 for t in trials if t['condition_type'] not in ('calibration', 'held_out_validation'))}")

    randomized = randomize_order(trials)
    save_sequence(randomized)

    print("\nFirst 10 trials in randomized run order (this is your actual testing sequence):")
    for t in sorted(randomized, key=lambda x: x["run_order"])[:10]:
        print(f"  #{t['run_order']:3d}: {t['controller']:12s} | {t['terrain']:14s} | "
              f"payload={t['payload_g']}g | {t['condition_type']}")

    print(f"\nIMPORTANT: This sequence was generated with RANDOM_SEED={RANDOM_SEED}.")
    print("Do not re-run this script with a different seed after data collection begins —")
    print("the saved trial_sequence.csv IS your pre-registered, locked testing order.")
