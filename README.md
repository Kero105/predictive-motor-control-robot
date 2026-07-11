# Predictive Electrical Feedback Control for Energy-Efficient Autonomous Mobile Robots

Independent research project comparing five motor control strategies on a custom-built 4WD robot, using real-time electrical, inertial, and proximity sensing to anticipate high-load terrain and obstacles before they fully develop, rather than only reacting after the fact.

Built and researched independently, targeting CSEF/LACSEF, ISEF, and related science competitions.

## Overview

Battery-powered mobile robots waste significant energy when their motor controllers can't anticipate changing terrain, inclines, and obstacles, they simply react after current spikes or battery voltage sags. This project builds a fully instrumented robot and directly compares five control strategies to test whether predictive electrical and proximity feedback can meaningfully reduce energy consumption per meter without sacrificing path accuracy or mission success.

## Controllers compared

1. **Fixed-speed** — no adaptation, baseline.
2. **PID** — closed-loop velocity regulation via encoder feedback.
3. **Threshold-based protection** — reactive slowdown after current/voltage exceeds a fixed threshold.
4. **Predictive** — a Kalman filter forecasts near-future current draw from a rolling window, and dual time-of-flight sensors anticipate obstacles, triggering preemptive speed reduction and steering bias before a load event or collision fully develops.
5. **Hybrid** — combines PID velocity regulation with the predictive layer modulating target speed, acceleration limits, and steering bias.

A trained decision-tree terrain classifier (current draw + accelerometer variance features) runs in parallel, distinguishing smooth, carpeted, and inclined surfaces in real time, evaluated against a hand-tuned rule-based baseline.

## Hardware

- ESP32-WROOM-32 microcontroller
- 4x Pololu 50:1 micro metal gearmotors with encoders (paired left/right, differential drive)
- 2x TB6612FNG motor drivers
- 2x INA226 current/voltage sensors
- MPU6050 IMU
- 2x VL53L0X time-of-flight distance sensors (front-facing + side-facing for steering)
- 10kΩ NTC thermistor (battery temperature)
- 2S LiPo battery (primary) + secondary lower-capacity 2S LiPo (low-battery edge-case testing)
- MicroSD card for onboard data logging

## Repository contents

| File | Description |
|---|---|
| `robot_firmware.ino` | Main ESP32 firmware: all five controllers, Kalman filter, sensor fusion, dual-ToF steering, SD logging. Includes a `TEST_MODE` flag for running/verifying control logic on the bare ESP32 with simulated sensor data, no other hardware required. |
| `terrain_classifier_trained.h` | Auto-generated decision tree terrain classifier, exported from `train_terrain_classifier.py`. Included directly by the firmware. |
| `train_terrain_classifier.py` | Trains the terrain classifier (currently on placeholder data) and exports it to embeddable C++. |
| `analyze_controllers.py` | Statistical analysis pipeline: ANOVA, pairwise t-tests, terrain classification accuracy, and obstacle-response analysis across controllers. |
| `energy_model.py` | Physics-based analytical energy model (rolling resistance + incline work), compared against measured energy consumption. |
| `randomization_script.py` | Generates the fixed, pre-registered randomized trial order used during data collection. |
| `experimental_protocol.md` | Full experimental protocol: variables, trial structure, randomization approach, statistical analysis plan. |
| `paper_draft.md` | Introduction, Literature Review, and Methods sections of the research paper. |

## Status

Firmware and analysis pipeline complete and tested (currently using simulated/placeholder data pending hardware arrival and bench-test calibration). Physical build and data collection in progress.

## License

MIT.
