"""
Physics-Based Energy Model
Predicts expected energy-per-meter from terrain classification, speed, and payload,
so you can compare MEASURED energy consumption against a theoretical prediction.
This is what lets you claim "our savings match/exceed what physics predicts,"
not just "our controller used less energy."

The model here is a simplified rolling-resistance + incline-work model.
You will need to CALIBRATE the resistance coefficients using your own bench-test
data (see calibrate_coefficients() stub) once you have real measurements —
the values below are reasonable starting placeholders, not measured constants.
"""

import numpy as np
import pandas as pd

# --- Physical constants (adjust to your actual robot) ---
ROBOT_MASS_KG = 0.35          # base robot mass, update with your actual measured mass
GRAVITY = 9.81

# Rolling resistance coefficients per terrain — PLACEHOLDERS, calibrate from bench data
# (typical literature ranges: smooth hard floor ~0.01-0.02, carpet ~0.05-0.15)
ROLLING_RESISTANCE_COEFF = {
    "smooth": 0.015,
    "carpet": 0.08,
    "ramp": 0.03,   # rolling resistance on the ramp surface itself; incline handled separately
}

RAMP_ANGLE_DEG = {
    "ramp": 10,        # your original ramp angle — update to match your actual build
    "ramp_2": 18,       # your second angle for held-out validation — update to match
}

MOTOR_EFFICIENCY = 0.65   # rough estimate for small brushed gearmotors; calibrate if possible


def predicted_energy_per_meter(terrain, payload_kg=0.0, ramp_key=None):
    """
    Returns predicted energy (in Joules) required to move 1 meter under the
    given conditions, based on rolling resistance + (if applicable) incline work.
    """
    total_mass = ROBOT_MASS_KG + payload_kg
    rr_coeff = ROLLING_RESISTANCE_COEFF.get(terrain, ROLLING_RESISTANCE_COEFF["smooth"])

    # Rolling resistance force = coefficient * normal force (mass * g, on flat ground)
    rolling_force_N = rr_coeff * total_mass * GRAVITY

    incline_work_J_per_m = 0.0
    if terrain == "ramp" and ramp_key:
        angle_rad = np.radians(RAMP_ANGLE_DEG[ramp_key])
        # Work to climb 1m of ramp surface distance = m*g*sin(theta) per meter traveled
        incline_work_J_per_m = total_mass * GRAVITY * np.sin(angle_rad)

    # Total mechanical work per meter, then account for motor efficiency losses
    mechanical_work_J_per_m = rolling_force_N + incline_work_J_per_m
    electrical_energy_J_per_m = mechanical_work_J_per_m / MOTOR_EFFICIENCY

    return electrical_energy_J_per_m


def joules_to_wh(joules):
    return joules / 3600.0


def compare_predicted_vs_measured(measured_df):
    """
    measured_df expects columns: terrain, payload_g, ramp_key (or None), energy_per_meter_measured (Wh/m)
    Returns a DataFrame with predicted values added alongside, plus percent error.
    """
    results = []
    for _, row in measured_df.iterrows():
        predicted_J = predicted_energy_per_meter(
            terrain=row["terrain"],
            payload_kg=row["payload_g"] / 1000.0,
            ramp_key=row.get("ramp_key", None),
        )
        predicted_Wh = joules_to_wh(predicted_J)
        measured_Wh = row["energy_per_meter_measured"]
        percent_error = ((measured_Wh - predicted_Wh) / predicted_Wh) * 100 if predicted_Wh != 0 else float("nan")

        results.append({
            **row,
            "predicted_Wh_per_m": round(predicted_Wh, 4),
            "measured_Wh_per_m": round(measured_Wh, 4),
            "percent_error": round(percent_error, 1),
        })
    return pd.DataFrame(results)


def calibrate_coefficients(bench_test_df):
    """
    STUB — once you've bench-tested each terrain individually (Phase 2 of your build:
    running a single motor at known speed on smooth floor, then carpet, then the ramp,
    measuring actual current/voltage), use those measurements to solve for the real
    rolling resistance coefficients instead of the placeholder values above.

    Expects bench_test_df with columns: terrain, measured_force_N (or derived from
    torque/current), known_mass_kg.
    Should return a dict of calibrated ROLLING_RESISTANCE_COEFF values.
    """
    raise NotImplementedError(
        "Fill this in using your Phase 2 bench-test current/voltage data per terrain."
    )


if __name__ == "__main__":
    # --- Example usage with dummy measured data, showing the comparison in action ---
    dummy_measured = pd.DataFrame([
        {"terrain": "smooth", "payload_g": 0, "ramp_key": None, "energy_per_meter_measured": 0.0009},
        {"terrain": "carpet", "payload_g": 0, "ramp_key": None, "energy_per_meter_measured": 0.0028},
        {"terrain": "ramp", "payload_g": 0, "ramp_key": "ramp", "energy_per_meter_measured": 0.0065},
        {"terrain": "smooth", "payload_g": 400, "ramp_key": None, "energy_per_meter_measured": 0.0014},
    ])

    comparison = compare_predicted_vs_measured(dummy_measured)
    print("=== Predicted vs. Measured Energy Comparison (dummy data) ===\n")
    print(comparison.to_string(index=False))
    print("\nNOTE: Replace dummy_measured with your real trial summary data once collected.")
    print("NOTE: Run calibrate_coefficients() with real bench-test data before trusting")
    print("      the predicted values numerically — current coefficients are literature")
    print("      placeholders, not measured constants for YOUR specific robot.")
