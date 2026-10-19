# Predictive Electrical Feedback Control for Mobile Robots

This repo has the firmware and analysis pipeline for a research project comparing five motor control strategies on a small 4WD robot. The question is whether a robot can use its own real-time current, voltage, inertial and distance readings to anticipate high-load terrain and obstacles, and slow down early enough to save energy per meter.

## Status

**The hardware build is paused.** I'm focusing on a new research project in RISC-V processor security.

- The firmware compiles for the ESP32. Its `TEST_MODE` runs the control logic on a bare ESP32 with simulated sensor values, but it hasn't been run on the finished robot.
- The terrain classifier is trained on **simulated** data. It has to be retrained on real sensor data before its accuracy means anything.
- The analysis scripts run on **dummy** data, so the pipeline is tested end to end. There are no real trial results yet.

## Controllers compared

1. **Fixed-speed:** no adaptation; this is the baseline.
2. **PID:** closed-loop velocity control from encoder feedback.
3. **Threshold:** slows down after current or voltage crosses a fixed limit, so it only reacts.
4. **Predictive:** a Kalman filter forecasts near-future current draw, and two time-of-flight sensors spot obstacles, so the robot slows down *before* a load spike or collision.
5. **Hybrid:** PID speed control, with the predictive layer adjusting target speed, acceleration and steering.

## Hardware design

- ESP32-WROOM-32
- 4x Pololu 50:1 micro metal gearmotors with encoders, and 2x TB6612FNG motor drivers
- 2x INA226 current/voltage sensors and an MPU6050 IMU
- 2x VL53L0X time-of-flight sensors: one facing forward, one facing the side
- A 10 kΩ NTC thermistor to measure battery temperature
- 2S LiPo battery and MicroSD logging

## Repository layout

| File | What it is |
| --- | --- |
| `Robot Firmware/` | ESP32 firmware. It contains all five controllers, the Kalman filter, sensor reading, ToF steering and SD-card logging, plus `terrain_classifier_trained.h`, the exported classifier. |
| `TrainTerrainClassifier.py` | Trains the decision-tree terrain classifier in scikit-learn and generates the C++ header. |
| `AnalyzeControllers.py` | ANOVA and pairwise t-tests across the controllers. |
| `EnergyModel.py` | A physics energy model (rolling resistance plus incline work). The coefficients are placeholders until bench calibration. |
| `RandomizationScript.py`, `trial_sequence.csv` | The pre-generated randomized trial order. |
| `experimental_protocol.md` | Variables, trial structure and the statistical analysis plan. |

The statistics helpers were later pulled out into a reusable package, [scifair-toolkit](https://github.com/Kero105/scifair-toolkit).

## Building the firmware

Open the `.ino` file in `Robot Firmware/` with the Arduino IDE with ESP32 board support. Install the libraries listed at the top of the file: INA226 (Rob Tillaart), Adafruit MPU6050, Adafruit VL53L0X and Adafruit Unified Sensor.

## License

MIT
