"""
Statistical Analysis Script — Controller Comparison
Compares energy-per-meter (and other metrics) across the five controllers
using ANOVA + pairwise t-tests. Currently populated with DUMMY data so the
pipeline is fully tested before real trial data exists.

TO USE WITH REAL DATA:
  Replace the `generate_dummy_data()` call in main() with:
      df = pd.read_csv("your_real_trial_summary.csv")
  Your real CSV should have one row per TRIAL (not per sensor sample) with columns:
      controller_mode, terrain, payload_g, energy_per_meter, path_error_m,
      completion_time_s, success (1/0)
  (You'll aggregate your raw per-sample logs into this summary format first —
   see the aggregate_raw_logs() helper stub at the bottom.)
"""

import pandas as pd
import numpy as np
from scipy import stats
import matplotlib.pyplot as plt

CONTROLLERS = ["fixed_speed", "pid", "threshold", "predictive", "hybrid"]


def generate_dummy_data(n_per_condition=10, seed=42):
    """Creates realistic-ish fake trial data so the analysis pipeline can be
    fully written and tested before real hardware data exists."""
    rng = np.random.default_rng(seed)
    rows = []
    # Rough dummy "true" energy-per-meter means per controller (arbitrary units, Wh/m)
    true_means = {
        "fixed_speed": 4.2,
        "pid": 3.8,
        "threshold": 3.6,
        "predictive": 3.1,
        "hybrid": 2.9,
    }
    terrains = ["smooth", "carpet", "ramp"]
    payloads = [0, 200, 400]

    for controller in CONTROLLERS:
        for terrain in terrains:
            for payload in payloads:
                for _ in range(n_per_condition):
                    noise = rng.normal(0, 0.4)
                    terrain_penalty = {"smooth": 0, "carpet": 0.6, "ramp": 1.0}[terrain]
                    payload_penalty = payload * 0.001
                    energy = true_means[controller] + terrain_penalty + payload_penalty + noise

                    # Dummy terrain classification: mostly correct, with some confusion
                    # between carpet/ramp (both elevate current, easy to conflate)
                    classifier_confusion = {
                        "smooth": ["smooth"] * 9 + ["carpet"],
                        "carpet": ["carpet"] * 8 + ["smooth", "ramp"],
                        "ramp": ["ramp"] * 9 + ["carpet"],
                    }
                    classified_terrain = rng.choice(classifier_confusion[terrain])

                    # Dummy obstacle-response data (only meaningful for predictive/hybrid,
                    # but generated for all controllers so the schema is consistent —
                    # fixed_speed/PID/threshold won't react, giving a realistic contrast)
                    obstacle_present = rng.random() > 0.7
                    if obstacle_present:
                        if controller in ("predictive", "hybrid"):
                            response_time_ms = rng.normal(120, 30)
                            avoided = rng.random() > 0.1
                        else:
                            response_time_ms = rng.normal(400, 80)  # no anticipation, reacts late
                            avoided = rng.random() > 0.4
                    else:
                        response_time_ms = np.nan
                        avoided = np.nan

                    rows.append({
                        "controller_mode": controller,
                        "terrain": terrain,
                        "terrain_classified": classified_terrain,
                        "payload_g": payload,
                        "energy_per_meter": max(energy, 0.1),
                        "path_error_m": abs(rng.normal(0.05, 0.02)),
                        "completion_time_s": rng.normal(15, 2),
                        "success": 1 if rng.random() > 0.05 else 0,
                        "obstacle_present": obstacle_present,
                        "obstacle_response_time_ms": response_time_ms,
                        "obstacle_avoided": avoided,
                    })
    return pd.DataFrame(rows)


def run_anova(df, metric="energy_per_meter"):
    groups = [df[df.controller_mode == c][metric].values for c in CONTROLLERS]
    f_stat, p_value = stats.f_oneway(*groups)
    print(f"\n=== One-way ANOVA on {metric} across controllers ===")
    print(f"F-statistic: {f_stat:.4f}   p-value: {p_value:.6f}")
    if p_value < 0.05:
        print("Result: statistically significant difference between controllers.")
    else:
        print("Result: NOT statistically significant at alpha=0.05.")
    return f_stat, p_value


def run_pairwise_ttests(df, metric="energy_per_meter"):
    print(f"\n=== Pairwise t-tests on {metric} (Bonferroni-corrected) ===")
    n_comparisons = len(CONTROLLERS) * (len(CONTROLLERS) - 1) // 2
    alpha_corrected = 0.05 / n_comparisons
    for i in range(len(CONTROLLERS)):
        for j in range(i + 1, len(CONTROLLERS)):
            a = df[df.controller_mode == CONTROLLERS[i]][metric]
            b = df[df.controller_mode == CONTROLLERS[j]][metric]
            t_stat, p_val = stats.ttest_ind(a, b)
            sig = "SIGNIFICANT" if p_val < alpha_corrected else "not significant"
            print(f"{CONTROLLERS[i]:12s} vs {CONTROLLERS[j]:12s}: "
                  f"t={t_stat:6.3f}  p={p_val:.5f}  ({sig}, corrected alpha={alpha_corrected:.5f})")


def summary_table(df, metric="energy_per_meter"):
    print(f"\n=== Summary statistics: {metric} ===")
    summary = df.groupby("controller_mode")[metric].agg(["mean", "std", "count"])
    summary = summary.reindex(CONTROLLERS)
    print(summary.round(3))
    return summary


def plot_comparison(df, metric="energy_per_meter", save_path="controller_comparison.png"):
    fig, ax = plt.subplots(figsize=(8, 5))
    data_by_controller = [df[df.controller_mode == c][metric].values for c in CONTROLLERS]
    ax.boxplot(data_by_controller, tick_labels=CONTROLLERS)
    ax.set_ylabel(metric.replace("_", " ").title())
    ax.set_title(f"{metric.replace('_', ' ').title()} by Controller")
    plt.xticks(rotation=15)
    plt.tight_layout()
    plt.savefig(save_path, dpi=150)
    print(f"\nSaved plot to {save_path}")
    plt.close()


def terrain_classification_accuracy(df):
    """
    Computes overall and per-terrain classification accuracy, plus a confusion matrix,
    comparing the classifier's real-time 'terrain_classified' output against the
    ground-truth 'terrain' label recorded per trial.
    """
    print("\n=== Terrain Classification Accuracy ===")
    df = df.copy()
    df["correct"] = df["terrain"] == df["terrain_classified"]
    overall_accuracy = df["correct"].mean()
    print(f"Overall accuracy: {overall_accuracy:.3f} ({overall_accuracy*100:.1f}%)")

    print("\nPer-terrain accuracy:")
    per_terrain = df.groupby("terrain")["correct"].mean()
    print(per_terrain.round(3))

    print("\nConfusion matrix (rows = true terrain, columns = classified terrain):")
    confusion = pd.crosstab(df["terrain"], df["terrain_classified"],
                             rownames=["True"], colnames=["Classified"])
    print(confusion)

    return overall_accuracy, confusion


def obstacle_response_analysis(df):
    """
    Compares obstacle-response time and avoidance success rate across controllers,
    for trials where an obstacle was actually present. Only predictive/hybrid
    controllers are expected to show fast (anticipatory) response times; the others
    serve as a reactive-only baseline for comparison.
    """
    print("\n=== Obstacle Response Analysis ===")
    obstacle_trials = df[df["obstacle_present"] == True].copy()

    if obstacle_trials.empty:
        print("No obstacle trials found in this dataset.")
        return None

    print(f"\nTotal obstacle-present trials: {len(obstacle_trials)}")

    print("\nMean response time (ms) by controller:")
    response_summary = obstacle_trials.groupby("controller_mode")["obstacle_response_time_ms"].agg(["mean", "std", "count"])
    response_summary = response_summary.reindex(CONTROLLERS)
    print(response_summary.round(1))

    print("\nAvoidance success rate by controller:")
    avoidance_summary = obstacle_trials.groupby("controller_mode")["obstacle_avoided"].mean()
    avoidance_summary = avoidance_summary.reindex(CONTROLLERS)
    print(avoidance_summary.round(3))

    # Statistical comparison: response time across controllers
    groups = [obstacle_trials[obstacle_trials.controller_mode == c]["obstacle_response_time_ms"].dropna().values
              for c in CONTROLLERS]
    groups = [g for g in groups if len(g) > 1]  # guard against empty/singleton groups
    if len(groups) >= 2:
        f_stat, p_value = stats.f_oneway(*groups)
        print(f"\nANOVA on obstacle response time: F={f_stat:.3f}  p={p_value:.6f}")

    # Chi-square test on avoidance success (predictive/hybrid vs. reactive controllers)
    anticipatory = obstacle_trials[obstacle_trials.controller_mode.isin(["predictive", "hybrid"])].dropna(subset=["obstacle_avoided"])
    reactive = obstacle_trials[obstacle_trials.controller_mode.isin(["fixed_speed", "pid", "threshold"])].dropna(subset=["obstacle_avoided"])
    anticipatory_avoided = anticipatory["obstacle_avoided"].astype(bool)
    reactive_avoided = reactive["obstacle_avoided"].astype(bool)
    if len(anticipatory) > 0 and len(reactive) > 0:
        contingency = pd.DataFrame({
            "avoided": [int(anticipatory_avoided.sum()), int(reactive_avoided.sum())],
            "not_avoided": [int((~anticipatory_avoided).sum()), int((~reactive_avoided).sum())],
        }, index=["anticipatory (predictive/hybrid)", "reactive (fixed/pid/threshold)"])
        chi2, p_val, dof, expected = stats.chi2_contingency(contingency)
        print(f"\nAnticipatory vs. reactive avoidance rate comparison:")
        print(contingency)
        print(f"Chi-square={chi2:.3f}  p={p_val:.6f}")

    return response_summary, avoidance_summary


def plot_obstacle_response(df, save_path="obstacle_response.png"):
    obstacle_trials = df[df["obstacle_present"] == True]
    if obstacle_trials.empty:
        return
    fig, ax = plt.subplots(figsize=(8, 5))
    data_by_controller = [obstacle_trials[obstacle_trials.controller_mode == c]["obstacle_response_time_ms"].dropna().values
                           for c in CONTROLLERS]
    ax.boxplot(data_by_controller, tick_labels=CONTROLLERS)
    ax.set_ylabel("Obstacle Response Time (ms)")
    ax.set_title("Obstacle Response Time by Controller")
    plt.xticks(rotation=15)
    plt.tight_layout()
    plt.savefig(save_path, dpi=150)
    print(f"\nSaved plot to {save_path}")
    plt.close()


def aggregate_raw_logs(raw_csv_path):
    """
    STUB — fill in once you have real per-sample SD card logs.
    Your ESP32 logs one row per ~20ms sample (voltage, current, encoders, etc.)
    This function should:
      1. Read the raw CSV
      2. Group by trial (you'll need a trial_id column, or split by timestamp gaps)
      3. Compute per-trial summary metrics:
         - energy_per_meter = integral(V*I dt) / distance_traveled
         - path_error_m = deviation from intended straight-line path
         - completion_time_s = last_timestamp - first_timestamp
         - success = 1 if trial completed course without cutoff, else 0
      4. Return a DataFrame in the summary format run_anova() etc. expect
    """
    raise NotImplementedError("Fill this in once you have real raw trial logs.")


def main():
    print("Running analysis pipeline with DUMMY data (replace with real CSV when ready)\n")
    df = generate_dummy_data()

    for metric in ["energy_per_meter", "path_error_m", "completion_time_s"]:
        summary_table(df, metric)
        run_anova(df, metric)

    run_pairwise_ttests(df, "energy_per_meter")
    plot_comparison(df, "energy_per_meter")

    print("\n=== Success rate by controller ===")
    print(df.groupby("controller_mode")["success"].mean().reindex(CONTROLLERS).round(3))

    terrain_classification_accuracy(df)
    obstacle_response_analysis(df)
    plot_obstacle_response(df)


if __name__ == "__main__":
    main()
