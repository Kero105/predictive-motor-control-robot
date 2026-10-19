# Experimental Protocol
## Predictive Electrical Feedback Control for Energy-Efficient Autonomous Mobile Robots

### 1. Purpose
This document specifies the exact experimental procedure, controlled variables, trial structure, and statistical analysis plan for this research project, prior to data collection, for pre-registration purposes.

### 2. Hypothesis
An energy-efficient (predictive or hybrid) motor controller will expend significantly less energy per meter traveled than fixed-speed, PID, or threshold-based reactive controllers, without a proportional loss in path accuracy or mission success rate.

### 3. Independent Variable
**Control algorithm**, with 5 levels:
1. Fixed-speed
2. PID
3. Threshold-based reactive protection
4. Predictive
5. Hybrid (PID + predictive)

### 4. Dependent Variables
- Energy consumption per meter (Wh/m), computed as ∫VI dt / distance
- Average current draw (A)
- Maximum current draw (A)
- Minimum battery voltage (V)
- Voltage drop (V)
- Run duration (s)
- Path deviation from intended straight-line course (m)
- Mission success (binary: completed course without cutoff/failure)
- Prediction precision and recall (predictive and hybrid controllers only)
- Terrain classification accuracy (against ground-truth terrain label per trial)
- Obstacle-detection response time and avoidance success rate (obstacle trials only)

### 5. Controlled Variables
- Robot hardware configuration (fixed for entire study, no component changes between trials)
- Initial battery voltage at the start of each trial (charged to the same target voltage before each session; verified with multimeter before each trial begins)
- Test course dimensions and layout (marked with painter's tape, fixed for the calibration terrain set)
- Motor/controller tuning parameters (fixed once calibrated during Phase 2 bench-testing; not re-tuned mid-session)
- Repetition count per condition (fixed at minimum 8-10 trials per condition, per power analysis below)
- Ambient conditions (trials conducted indoors, same location, to minimize wind/temperature variability)

### 6. Terrain Conditions
**Calibration set** (used for controller tuning and initial data collection):
- Smooth flooring (hard, level surface)
- Carpet Sample A
- Ramp at fixed Angle 1

**Held-out validation set** (used only for generalization testing, never used to tune/calibrate any controller):
- Carpet Sample B (different pile/thickness than Sample A)
- Ramp at fixed Angle 2 (different angle than Angle 1)

### 7. Payload Conditions
Payload achieved using a fixed, sealed container filled with precisely weighed rice (measured via kitchen scale before each trial). Payload levels: 0g, 200g, 400g (adjust based on chassis capacity confirmed during build).

### 8. Edge-Case / Robustness Conditions
- **Low-battery trials**: a subset of trials conducted using a secondary, lower-capacity (800mAh) 2S LiPo battery, run down closer to the low-voltage cutoff, to evaluate controller behavior under degraded power conditions.
- **Unanticipated obstacle trials**: a subset of trials introducing a previously uncalibrated obstacle (e.g., a small block) not present during controller tuning, to assess robustness beyond trained conditions.

### 9. Trial Structure
For each of the 5 controllers, trials are run across:
- 3 calibration terrain conditions × 3 payload levels = 9 base conditions
- Plus 2 held-out terrain conditions (at a single representative payload level) for generalization testing
- Plus edge-case conditions (low-battery, unanticipated obstacle) at a single representative terrain/payload combination

Each unique (controller × condition) combination is repeated a minimum of 8-10 times (final count determined by power analysis, see Section 11).

### 10. Randomization
Trial order is fully randomized across controller, terrain, and payload combinations for the calibration data collection phase, using a pre-generated randomized sequence (see accompanying `randomization_script.py`) rather than being determined ad hoc during testing. This controls for confounding factors that could otherwise correlate with trial order, including:
- Battery degradation across a testing session
- Motor wear/heating across repeated runs
- Any drift in ambient conditions during a session

The randomized sequence is generated and fixed **before** data collection begins, and is followed exactly as generated; deviations (e.g., skipped trials due to hardware failure) are logged with a reason.

### 11. Statistical Power Analysis
Prior to full data collection, a small set of dry-run trials (not included in final analysis) will be used to estimate preliminary variance in the primary outcome measure (energy per meter) for each controller. This variance estimate will be used in G*Power (or equivalent) to calculate the minimum sample size per condition required to detect a meaningful effect size (target: medium effect size, f = 0.25, alpha = 0.05, power = 0.80) via one-way ANOVA. The final repetition count per condition will be set to at least this calculated minimum, and no fewer than 8 repetitions regardless.

### 12. Statistical Analysis Plan
- One-way ANOVA across the 5 controllers for each continuous dependent variable (energy per meter, path deviation, completion time).
- Where the ANOVA is significant (p < 0.05), Bonferroni-corrected pairwise t-tests between all controller pairs.
- Mission success rate compared across controllers using chi-square test of independence.
- For predictive/hybrid controllers, precision and recall of load-event prediction calculated directly from logged predicted vs. actual load-event flags.
- Physics-based energy model predictions compared against measured energy-per-meter values, reporting percent error, to assess whether observed energy savings align with theoretical expectations.
- Trained decision-tree terrain classifier accuracy compared directly against the hand-tuned rule-based classifier (same input features), evaluated on identical held-out trial data.
- Kalman-filter-based current forecasting compared against the parallel least-squares regression forecast, evaluated by forecast error (predicted vs. actual current several samples later) rather than only by downstream control outcomes.
- Held-out terrain results analyzed separately from calibration-terrain results to explicitly test generalization, rather than pooling all terrain data together.

### 13. Data Collection Procedure (per trial)
1. Confirm battery charged to target starting voltage (verify with multimeter).
2. Set correct controller mode via Serial command.
3. Position robot at marked starting point on the assigned terrain condition.
4. Attach assigned payload (if applicable), confirm exact mass via kitchen scale, log the mass.
5. Begin logging (automatic on power-up per firmware); start trial.
6. Robot completes course (or trial is terminated if mission fails / safety cutoff triggers).
7. Record trial outcome (success/failure) and any observational notes.
8. Retrieve SD card log for that trial; back up data to computer.
9. Proceed to next trial per the pre-generated randomized sequence.

### 14. Deviations from Protocol
Any deviation from this protocol during actual data collection (equipment failure, environmental disruption, trial skipped or repeated) will be logged with the trial ID, reason, and resolution, and disclosed in the final paper's Methods/Limitations section.
