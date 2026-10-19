/*
  Predictive Electrical Feedback Control - Main Firmware
  ESP32-WROOM-32 DevKitC

  PIN PLAN (avoids strapping pins 0/2/12/15, UART pins 1/3, flash pins 6-11)
  ---------------------------------------------------------------
  I2C (INA226 x2 + MPU6050 + VL53L0X x2):     SDA=21, SCL=22
  SPI (SD card):                 SCK=18, MISO=19, MOSI=23, CS=5
  ToF #2 address-assignment control:          XSHUT=13
  Thermistor (voltage divider):  GPIO36 (ADC1_CH0)
  Motor driver STBY (shared):    GPIO4
  Left motor pair (FL+RL wired in parallel on TB6612 #1, channel A):
      DIR1=16, DIR2=17, PWM=25
  Right motor pair (FR+RR wired in parallel on TB6612 #2, channel A):
      DIR1=26, DIR2=27, PWM=14
  Left-side encoder (representative for both left motors):
      ENC_L_A=32, ENC_L_B=33
  Right-side encoder (representative for both right motors):
      ENC_R_A=34, ENC_R_B=35   (input-only pins, fine for encoder input)

  INA226 I2C addresses: Board 1 (main battery line) = 0x40, Board 2 (motor branch) = 0x41
  MPU6050 I2C address: 0x68 (default)
*/

#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <INA226.h>           // Rob Tillaart INA226 library
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_VL53L0X.h> // ToF distance sensor - shares existing I2C bus

// ---------------- TEST MODE ----------------
// Set to 1 to run the full control-logic pipeline with SIMULATED sensor data,
// bypassing real INA226/MPU6050/SD hardware entirely. Lets you verify controller
// behavior on just the bare ESP32 before motors/sensors arrive.
// Set to 0 for real hardware operation.
#define TEST_MODE 1

// ---------------- PIN DEFINITIONS ----------------
#define PIN_SD_CS      5
#define PIN_THERMISTOR 36
#define PIN_STBY       4

#define PIN_L_DIR1     16
#define PIN_L_DIR2     17
#define PIN_L_PWM      25

#define PIN_R_DIR1     26
#define PIN_R_DIR2     27
#define PIN_R_PWM      14

#define PIN_ENC_L_A    32
#define PIN_ENC_L_B    33
#define PIN_ENC_R_A    34
#define PIN_ENC_R_B    35

#define PIN_TOF2_XSHUT 13   // controls power-up/address assignment for the SECOND VL53L0X only;
                             // the first ToF sensor stays at its default address and needs no XSHUT pin

// ---------------- CUSTOM TYPES (must be declared here, before any function uses them) ----------------
// Arduino auto-generates function prototypes near the top of the file. If a custom type
// (enum/struct) used in a function's signature is defined LATER in the file, that
// auto-generated prototype fails to compile. Defining these types early avoids that.

enum TerrainClass { TERRAIN_SMOOTH, TERRAIN_CARPET, TERRAIN_RAMP, TERRAIN_UNKNOWN };
const char* terrainClassName(TerrainClass t) {
  switch (t) {
    case TERRAIN_SMOOTH: return "smooth";
    case TERRAIN_CARPET: return "carpet";
    case TERRAIN_RAMP: return "ramp";
    default: return "unknown";
  }
}

enum ControllerMode {
  FIXED_SPEED,
  PID_CONTROL,
  THRESHOLD_PROTECTION,
  PREDICTIVE,
  HYBRID
};

struct SensorData {
  float voltage;
  float current;
  long encL, encR;
  float ax, ay, az;
  float gx, gy, gz;
  float distance_mm;       // front ToF - straight-ahead obstacle detection
  float distance_side_mm;  // side ToF - used for steering-direction decisions
};

// ---------------- EXPLICIT FORWARD DECLARATIONS ----------------
// Arduino's automatic prototype generation is unreliable for functions whose
// signature uses a custom struct/enum type. Declaring these explicitly here,
// right after the types are defined, avoids depending on that auto-generation
// working correctly.
SensorData readSensors();
SensorData simulateSensors();
bool controller_thresholdCheck(SensorData &d);
void controller_thresholdProtection(float &leftOut, float &rightOut, SensorData &d);
void controller_predictive(float &leftOut, float &rightOut, bool &predictedEvent, SensorData &d);
void controller_hybrid(float &leftOut, float &rightOut, bool &predictedEvent, SensorData &d, float velL, float velR, float dt);
void logRow(SensorData &d, ControllerMode mode, bool predictedEvent, bool actualEvent, TerrainClass terrain);
bool updatePredictiveModel(float currentReading, float distance_front_mm, float distance_side_mm);
float updateAccelVariance(float ax, float ay, float az);
TerrainClass classifyTerrain(float avgCurrent, float accelVariance);
TerrainClass classifyTerrain_ruleBased(float avgCurrent, float accelVariance);

// >>> SET WHICH CONTROLLER TO RUN FOR THIS TRIAL <<<
ControllerMode currentMode = FIXED_SPEED;

// ---------------- SENSOR OBJECTS ----------------
INA226 ina_battery(0x40);   // main battery line
INA226 ina_motor(0x41);     // motor branch
Adafruit_MPU6050 mpu;
Adafruit_VL53L0X tof_front = Adafruit_VL53L0X();  // I2C addr 0x29 (default) - straight-ahead obstacle detection
Adafruit_VL53L0X tof_side = Adafruit_VL53L0X();   // I2C addr 0x30 (reassigned via XSHUT) - angled sensor for steering decisions

// ---------------- ENCODER STATE ----------------
volatile long encoderCountL = 0;
volatile long encoderCountR = 0;

// Counts per revolution of the OUTPUT SHAFT for the 50:1 micro metal gearmotor
// (12 CPR at motor shaft * 51.45:1 gear ratio ≈ 617 CPR at output shaft)
const float COUNTS_PER_REV = 617.0;
const float WHEEL_DIAMETER_MM = 70.0;
const float WHEEL_CIRCUMFERENCE_M = (WHEEL_DIAMETER_MM / 1000.0) * PI;

void IRAM_ATTR encoderISR_L() {
  bool a = digitalRead(PIN_ENC_L_A);
  bool b = digitalRead(PIN_ENC_L_B);
  encoderCountL += (a == b) ? 1 : -1;
}

void IRAM_ATTR encoderISR_R() {
  bool a = digitalRead(PIN_ENC_R_A);
  bool b = digitalRead(PIN_ENC_R_B);
  encoderCountR += (a == b) ? 1 : -1;
}

// ---------------- CONTROL TARGETS / TUNING ----------------
float targetSpeed_mps = 0.3;     // baseline target speed, meters/sec — tune per trial
float maxPWM = 255.0;

// PID gains — placeholders, tune once hardware is running
float Kp = 40.0, Ki = 5.0, Kd = 2.0;
float integralL = 0, integralR = 0;
float lastErrorL = 0, lastErrorR = 0;

// Threshold-protection limits
const float CURRENT_THRESHOLD_A = 1.0;   // trip point, tune from bench-test data
const float VOLTAGE_MIN_V = 6.5;         // low-voltage cutoff for 2S LiPo

// Predictive: ONLINE LINEAR REGRESSION over a rolling window of current draw,
// predicting near-future current from recent trend rather than a fixed derivative threshold.
const int ROLLING_WINDOW = 10;
float currentHistory[ROLLING_WINDOW];
int historyIndex = 0;
bool historyFilled = false;
const float PREDICTED_CURRENT_THRESHOLD_A = 0.9;  // tune from bench data: predicted-ahead current that triggers a slowdown
float predictSpeedFactor = 1.0;  // 1.0 = full speed, reduced when high load predicted
float lastRegressionPredictedCurrent = 0;

// --- Terrain classification ---
// Simple rule-based classifier using current draw baseline + IMU vibration signature.
// TODO once real bench data exists: replace fixed thresholds below with values
// measured during Phase 2 bench-testing on each actual terrain (smooth/carpet/ramp).
// Trained decision-tree classifier, generated by train_terrain_classifier.py.
// Regenerate that header once you have REAL bench-test (current, accel_variance, terrain)
// data, then re-include it here — this replaces the hand-tuned rule-based classifier.
#include "terrain_classifier_trained.h"
#define USE_TRAINED_CLASSIFIER 1  // 1 = use the trained tree, 0 = fall back to hand-tuned rules

// ---------------- KALMAN FILTER (current-draw trend estimation) ----------------
// 1D Kalman filter tracking [current_level, current_rate_of_change] as state,
// giving a more principled forecast than the plain linear-regression predictor
// by explicitly modeling process and measurement noise rather than just fitting a line.
struct KalmanState {
  float level;       // estimated current current draw (A)
  float trend;        // estimated rate of change of current (A per sample)
  float p[2][2];       // state covariance matrix
};

KalmanState kalman = { 0.3, 0.0, {{1, 0}, {0, 1}} };

// Tuning knobs — how much we trust the model vs. new measurements.
// Larger Q = trust measurements more (react faster, noisier). Larger R = trust model more (smoother, slower).
const float KALMAN_Q_LEVEL = 0.001;
const float KALMAN_Q_TREND = 0.0005;
const float KALMAN_R = 0.02;

float kalmanUpdate(float measuredCurrent) {
  // --- Predict step ---
  float predictedLevel = kalman.level + kalman.trend;
  float predictedTrend = kalman.trend;

  float p00 = kalman.p[0][0] + kalman.p[0][1] + kalman.p[1][0] + kalman.p[1][1] + KALMAN_Q_LEVEL;
  float p01 = kalman.p[0][1] + kalman.p[1][1];
  float p10 = kalman.p[1][0] + kalman.p[1][1];
  float p11 = kalman.p[1][1] + KALMAN_Q_TREND;

  // --- Update step (incorporate the new measurement) ---
  float innovation = measuredCurrent - predictedLevel;
  float s = p00 + KALMAN_R; // innovation covariance
  float k0 = p00 / s;       // Kalman gain for level
  float k1 = p10 / s;       // Kalman gain for trend

  kalman.level = predictedLevel + k0 * innovation;
  kalman.trend = predictedTrend + k1 * innovation;

  kalman.p[0][0] = (1 - k0) * p00;
  kalman.p[0][1] = (1 - k0) * p01;
  kalman.p[1][0] = p10 - k1 * p00;
  kalman.p[1][1] = p11 - k1 * p01;

  return kalman.level;
}

// Forecasts current N samples ahead using the Kalman filter's estimated trend.
float kalmanPredictCurrent(int samplesAhead) {
  return kalman.level + kalman.trend * samplesAhead;
}

// Least-squares linear regression over the rolling current window.
// Returns the predicted current N samples into the future (extrapolating the fitted line).
float regressionPredictCurrent(int samplesAhead) {
  if (!historyFilled) return currentHistory[historyIndex == 0 ? ROLLING_WINDOW - 1 : historyIndex - 1];

  float sumX = 0, sumY = 0, sumXY = 0, sumX2 = 0;
  for (int i = 0; i < ROLLING_WINDOW; i++) {
    // reconstruct chronological order: oldest sample = x=0, newest = x=WINDOW-1
    int idx = (historyIndex + i) % ROLLING_WINDOW;
    float x = i;
    float y = currentHistory[idx];
    sumX += x; sumY += y; sumXY += x * y; sumX2 += x * x;
  }
  float n = ROLLING_WINDOW;
  float denom = (n * sumX2 - sumX * sumX);
  if (fabs(denom) < 1e-6) return sumY / n; // fallback: flat, no discernible trend

  float slope = (n * sumXY - sumX * sumY) / denom;
  float intercept = (sumY - slope * sumX) / n;

  float futureX = (n - 1) + samplesAhead;
  return intercept + slope * futureX;
}

// Simple rule-based terrain classifier using average current + accel variance.
// Placeholder thresholds -- calibrate with real bench-test data (Phase 2).
TerrainClass classifyTerrain_ruleBased(float avgCurrent, float accelVariance) {
  if (accelVariance > 2.0) return TERRAIN_RAMP;       // large orientation change = incline
  if (avgCurrent > 0.6) return TERRAIN_CARPET;         // elevated steady current = higher-friction surface
  return TERRAIN_SMOOTH;
}

// Wrapper: switches between the trained decision tree and the hand-tuned rule-based
// fallback via USE_TRAINED_CLASSIFIER. Use the trained version once real bench data
// has been used to retrain it (see train_terrain_classifier.py).
TerrainClass classifyTerrain(float avgCurrent, float accelVariance) {
#if USE_TRAINED_CLASSIFIER
  return classifyTerrain_trained(avgCurrent, accelVariance);
#else
  return classifyTerrain_ruleBased(avgCurrent, accelVariance);
#endif
}

// Rolling accel-magnitude buffer for computing variance (used by terrain classifier)
const int ACCEL_WINDOW = 10;
float accelMagHistory[ACCEL_WINDOW];
int accelHistoryIndex = 0;
bool accelHistoryFilled = false;

float updateAccelVariance(float ax, float ay, float az) {
  float mag = sqrt(ax * ax + ay * ay + az * az);
  accelMagHistory[accelHistoryIndex] = mag;
  accelHistoryIndex = (accelHistoryIndex + 1) % ACCEL_WINDOW;
  if (accelHistoryIndex == 0) accelHistoryFilled = true;

  int count = accelHistoryFilled ? ACCEL_WINDOW : accelHistoryIndex;
  if (count < 2) return 0;

  float mean = 0;
  for (int i = 0; i < count; i++) mean += accelMagHistory[i];
  mean /= count;

  float variance = 0;
  for (int i = 0; i < count; i++) variance += pow(accelMagHistory[i] - mean, 2);
  variance /= count;
  return variance;
}

// ---------------- LOGGING ----------------
File logFile;
unsigned long trialStartMillis = 0;

void setup() {
  Serial.begin(115200);
  Wire.begin(21, 22);

  // Motor pins
  pinMode(PIN_STBY, OUTPUT);
  pinMode(PIN_L_DIR1, OUTPUT);
  pinMode(PIN_L_DIR2, OUTPUT);
  pinMode(PIN_R_DIR1, OUTPUT);
  pinMode(PIN_R_DIR2, OUTPUT);
  digitalWrite(PIN_STBY, HIGH); // enable both TB6612 boards

  // PWM setup (ESP32 LEDC)
  ledcAttach(PIN_L_PWM, 5000, 8);  // 5kHz, 8-bit resolution
  ledcAttach(PIN_R_PWM, 5000, 8);

  // Encoders
  pinMode(PIN_ENC_L_A, INPUT);
  pinMode(PIN_ENC_L_B, INPUT);
  pinMode(PIN_ENC_R_A, INPUT);
  pinMode(PIN_ENC_R_B, INPUT);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_L_A), encoderISR_L, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_R_A), encoderISR_R, CHANGE);

#if !TEST_MODE
  // INA226 sensors
  ina_battery.begin();
  ina_battery.setMaxCurrentShunt(2.0, 0.1); // calibrate for ~2A max, adjust to your shunt
  ina_motor.begin();
  ina_motor.setMaxCurrentShunt(2.0, 0.1);

  // MPU6050
  if (!mpu.begin()) {
    Serial.println("MPU6050 not found - check wiring");
  }
  mpu.setAccelerometerRange(MPU6050_RANGE_4_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);

  // VL53L0X ToF distance sensors - dual sensor address assignment via XSHUT
  // Sequence: hold sensor #2 in reset (XSHUT low) while #1 initializes at its default
  // address, then release #2 and reassign it to a different address before it wakes up.
  pinMode(PIN_TOF2_XSHUT, OUTPUT);
  digitalWrite(PIN_TOF2_XSHUT, LOW); // keep sensor #2 held in reset/shutdown

  delay(10);
  if (!tof_front.begin(0x29)) {
    Serial.println("Front VL53L0X not found - check wiring");
  }

  digitalWrite(PIN_TOF2_XSHUT, HIGH); // release sensor #2 from reset
  delay(10);
  if (!tof_side.begin(0x30)) {
    Serial.println("Side VL53L0X not found - check wiring/address assignment");
  }

  // SD card
  if (!SD.begin(PIN_SD_CS)) {
    Serial.println("SD card init failed");
  } else {
    logFile = SD.open("/trial_log.csv", FILE_APPEND);
    if (logFile) {
      logFile.println("timestamp_ms,voltage,current,encoder_L,encoder_R,accel_x,accel_y,accel_z,gyro_x,gyro_y,gyro_z,distance_front_mm,distance_side_mm,terrain_classified,regression_predicted_current,kalman_predicted_current,steer_bias,controller_mode,predicted_load_event,actual_load_event");
      logFile.close();
    }
  }
#else
  Serial.println("*** TEST_MODE ENABLED — using simulated sensor data, no real hardware required ***");
  Serial.println("Send a character over Serial to change controller mode:");
  Serial.println("  f = Fixed Speed | p = PID | t = Threshold | r = Predictive | h = Hybrid");
#endif

  trialStartMillis = millis();
  Serial.println("Setup complete.");
}

// ---------------- SIMULATED SENSOR DATA (TEST_MODE only) ----------------
// Generates a current draw that ramps up, spikes (simulating hitting carpet/incline),
// then settles — so you can watch the predictive controller react to it live over Serial.
SensorData simulateSensors() {
  SensorData d;
  static unsigned long simStart = millis();
  float t = (millis() - simStart) / 1000.0; // seconds since boot

  // Simulate a current draw that oscillates with an occasional upward spike every ~6 seconds
  float baseCurrent = 0.3 + 0.1 * sin(t);
  float spike = (fmod(t, 6.0) > 4.5) ? (fmod(t, 6.0) - 4.5) * 0.8 : 0.0; // ramps up before a "load event"
  d.current = baseCurrent + spike;

  d.voltage = 7.6 - (d.current * 0.15); // simple simulated sag under load
  d.encL = encoderCountL; // still real if you spin a test motor by hand, else 0
  d.encR = encoderCountR;
  d.ax = 0; d.ay = 0; d.az = 9.8;
  d.gx = 0; d.gy = 0; d.gz = 0;
  // Simulate an obstacle drifting closer every ~10 seconds, then resetting
  float obstacleCycle = fmod(t, 10.0);
  d.distance_mm = (obstacleCycle < 7.0) ? (2000 - obstacleCycle * 250) : 9999;
  // Simulate the side sensor seeing a clear path about half the time (alternating),
  // so you can watch the steering bias flip direction in the Serial output
  d.distance_side_mm = (fmod(t, 20.0) < 10.0) ? 9999 : 100;
  return d;
}

// ---------------- SERIAL MODE SWITCHING ----------------
void checkSerialForModeChange() {
  if (Serial.available() > 0) {
    char c = Serial.read();
    switch (c) {
      case 'f': currentMode = FIXED_SPEED; Serial.println(">> Mode: FIXED_SPEED"); break;
      case 'p': currentMode = PID_CONTROL; Serial.println(">> Mode: PID_CONTROL"); break;
      case 't': currentMode = THRESHOLD_PROTECTION; Serial.println(">> Mode: THRESHOLD_PROTECTION"); break;
      case 'r': currentMode = PREDICTIVE; Serial.println(">> Mode: PREDICTIVE"); break;
      case 'h': currentMode = HYBRID; Serial.println(">> Mode: HYBRID"); break;
      default: break; // ignore newlines/unrecognized chars
    }
    // Reset controller state on mode change so old integral/history doesn't carry over
    integralL = 0; integralR = 0;
    lastErrorL = 0; lastErrorR = 0;
    historyIndex = 0; historyFilled = false;
    predictSpeedFactor = 1.0;
  }
}

SensorData readSensors() {
  SensorData d;
  d.voltage = ina_battery.getBusVoltage();
  d.current = ina_battery.getCurrent_mA() / 1000.0;
  d.encL = encoderCountL;
  d.encR = encoderCountR;

  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  d.ax = a.acceleration.x;
  d.ay = a.acceleration.y;
  d.az = a.acceleration.z;
  d.gx = g.gyro.x;
  d.gy = g.gyro.y;
  d.gz = g.gyro.z;

  VL53L0X_RangingMeasurementData_t measureFront, measureSide;
  tof_front.rangingTest(&measureFront, false);
  d.distance_mm = (measureFront.RangeStatus != 4) ? measureFront.RangeMilliMeter : 9999;

  tof_side.rangingTest(&measureSide, false);
  d.distance_side_mm = (measureSide.RangeStatus != 4) ? measureSide.RangeMilliMeter : 9999;

  return d;
}

// ---------------- MOTOR CONTROL ----------------
void setMotors(float leftPWM, float rightPWM) {
  // leftPWM/rightPWM range: -255 to 255 (sign = direction)
  digitalWrite(PIN_L_DIR1, leftPWM >= 0 ? HIGH : LOW);
  digitalWrite(PIN_L_DIR2, leftPWM >= 0 ? LOW : HIGH);
  ledcWrite(PIN_L_PWM, (int)abs(leftPWM));

  digitalWrite(PIN_R_DIR1, rightPWM >= 0 ? HIGH : LOW);
  digitalWrite(PIN_R_DIR2, rightPWM >= 0 ? LOW : HIGH);
  ledcWrite(PIN_R_PWM, (int)abs(rightPWM));
}

// ---------------- VELOCITY CALC ----------------
long lastEncL = 0, lastEncR = 0;
unsigned long lastVelTime = 0;

void getWheelVelocities(float &velL_mps, float &velR_mps) {
  unsigned long now = millis();
  float dt = (now - lastVelTime) / 1000.0;
  if (dt <= 0) dt = 0.001;

  long dEncL = encoderCountL - lastEncL;
  long dEncR = encoderCountR - lastEncR;

  velL_mps = (dEncL / COUNTS_PER_REV) * WHEEL_CIRCUMFERENCE_M / dt;
  velR_mps = (dEncR / COUNTS_PER_REV) * WHEEL_CIRCUMFERENCE_M / dt;

  lastEncL = encoderCountL;
  lastEncR = encoderCountR;
  lastVelTime = now;
}

// ---------------- CONTROLLERS ----------------

void controller_fixedSpeed(float &leftOut, float &rightOut) {
  float pwm = (targetSpeed_mps / 0.5) * maxPWM; // rough scale, calibrate 0.5 m/s = max
  leftOut = pwm;
  rightOut = pwm;
}

float computePID(float target, float actual, float &integral, float &lastError, float dt) {
  float error = target - actual;
  integral += error * dt;
  integral = constrain(integral, -100, 100); // anti-windup
  float derivative = (error - lastError) / dt;
  lastError = error;
  float output = Kp * error + Ki * integral + Kd * derivative;
  return constrain(output, -maxPWM, maxPWM);
}

void controller_PID(float &leftOut, float &rightOut, float velL, float velR, float dt) {
  leftOut = computePID(targetSpeed_mps, velL, integralL, lastErrorL, dt);
  rightOut = computePID(targetSpeed_mps, velR, integralR, lastErrorR, dt);
}

bool controller_thresholdCheck(SensorData &d) {
  // Returns true if a protective action was taken
  if (d.current > CURRENT_THRESHOLD_A || d.voltage < VOLTAGE_MIN_V) {
    return true;
  }
  return false;
}

void controller_thresholdProtection(float &leftOut, float &rightOut, SensorData &d) {
  controller_fixedSpeed(leftOut, rightOut); // baseline behavior
  if (controller_thresholdCheck(d)) {
    leftOut *= 0.5;   // reactive cutback after overload detected
    rightOut *= 0.5;
  }
}

// Predictive: KALMAN FILTER is the primary current-draw forecaster (regression is
// still computed and logged alongside it, for direct comparison in your results).
// Also incorporates dual-ToF obstacle sensing: front sensor triggers anticipatory
// slowdown, side sensor determines which direction has more clearance for steering.
float steerBias = 0.0; // -1.0 = steer hard left, +1.0 = steer hard right, 0 = straight

bool updatePredictiveModel(float currentReading, float distance_front_mm, float distance_side_mm) {
  currentHistory[historyIndex] = currentReading;
  historyIndex = (historyIndex + 1) % ROLLING_WINDOW;
  if (historyIndex == 0) historyFilled = true;

  // Regression prediction - kept for logging/comparison against the Kalman approach
  lastRegressionPredictedCurrent = regressionPredictCurrent(3);

  // Kalman prediction - PRIMARY signal used for the actual control decision
  kalmanUpdate(currentReading);
  float kalmanPredictedCurrent = kalmanPredictCurrent(3);

  bool predictedLoadEvent = false;
  bool obstacleAhead = (distance_front_mm < 150.0); // tune based on robot speed/stopping distance
  steerBias = 0.0;

  if (obstacleAhead) {
    predictedLoadEvent = true;
    predictSpeedFactor = 0.3; // aggressive slowdown for a physical obstacle
    // Steering decision: side sensor tells us which way has more room.
    // If the side sensor ALSO reads close, there's an obstacle on that side too -
    // steer away from it. If the side sensor reads far, that side is clear - steer toward it.
    if (distance_side_mm < 150.0) {
      steerBias = -0.6;  // side is also blocked - steer away from it (turn opposite)
    } else {
      steerBias = 0.4;   // side is clear - bias steering toward the open side
    }
  } else if (historyFilled && (kalmanPredictedCurrent > PREDICTED_CURRENT_THRESHOLD_A)) {
    predictedLoadEvent = true;
    predictSpeedFactor = 0.6;  // preemptively reduce speed for anticipated electrical load
  } else if (predictSpeedFactor < 1.0) {
    predictSpeedFactor = min(1.0f, predictSpeedFactor + 0.02f); // gradual recovery
  }
  return predictedLoadEvent;
}

void controller_predictive(float &leftOut, float &rightOut, bool &predictedEvent, SensorData &d) {
  predictedEvent = updatePredictiveModel(d.current, d.distance_mm, d.distance_side_mm);
  controller_fixedSpeed(leftOut, rightOut);
  leftOut *= predictSpeedFactor;
  rightOut *= predictSpeedFactor;
  // Apply steering bias by differentially adjusting each side's speed
  leftOut *= (1.0 - steerBias);
  rightOut *= (1.0 + steerBias);
}

void controller_hybrid(float &leftOut, float &rightOut, bool &predictedEvent, SensorData &d, float velL, float velR, float dt) {
  predictedEvent = updatePredictiveModel(d.current, d.distance_mm, d.distance_side_mm);
  float scaledTarget = targetSpeed_mps * predictSpeedFactor;
  leftOut = computePID(scaledTarget * (1.0 - steerBias), velL, integralL, lastErrorL, dt);
  rightOut = computePID(scaledTarget * (1.0 + steerBias), velR, integralR, lastErrorR, dt);
}

// ---------------- LOGGING ----------------
void logRow(SensorData &d, ControllerMode mode, bool predictedEvent, bool actualEvent, TerrainClass terrain) {
  logFile = SD.open("/trial_log.csv", FILE_APPEND);
  if (logFile) {
    logFile.print(millis() - trialStartMillis); logFile.print(",");
    logFile.print(d.voltage); logFile.print(",");
    logFile.print(d.current); logFile.print(",");
    logFile.print(d.encL); logFile.print(",");
    logFile.print(d.encR); logFile.print(",");
    logFile.print(d.ax); logFile.print(",");
    logFile.print(d.ay); logFile.print(",");
    logFile.print(d.az); logFile.print(",");
    logFile.print(d.gx); logFile.print(",");
    logFile.print(d.gy); logFile.print(",");
    logFile.print(d.gz); logFile.print(",");
    logFile.print(d.distance_mm); logFile.print(",");
    logFile.print(d.distance_side_mm); logFile.print(",");
    logFile.print(terrainClassName(terrain)); logFile.print(",");
    logFile.print(lastRegressionPredictedCurrent); logFile.print(",");
    logFile.print(kalman.level + kalman.trend * 3); logFile.print(",");
    logFile.print(steerBias); logFile.print(",");
    logFile.print(mode); logFile.print(",");
    logFile.print(predictedEvent ? 1 : 0); logFile.print(",");
    logFile.println(actualEvent ? 1 : 0);
    logFile.close();
  }
}

// ---------------- MAIN LOOP ----------------
unsigned long lastLoopTime = 0;

void loop() {
  checkSerialForModeChange();

  unsigned long now = millis();
  float dt = (now - lastLoopTime) / 1000.0;
  if (dt <= 0) dt = 0.02; // fallback ~50Hz
  lastLoopTime = now;

#if TEST_MODE
  SensorData d = simulateSensors();
#else
  SensorData d = readSensors();
#endif

  float velL, velR;
  getWheelVelocities(velL, velR);

  float accelVariance = updateAccelVariance(d.ax, d.ay, d.az);
  TerrainClass terrain = classifyTerrain(d.current, accelVariance);

  float leftOut = 0, rightOut = 0;
  bool predictedEvent = false;
  // "Actual" high-load event, for precision/recall scoring against your prediction —
  // defined here as current exceeding the same threshold used for protection;
  // tune this definition once you have real bench-test current data.
  bool actualEvent = (d.current > CURRENT_THRESHOLD_A);

  switch (currentMode) {
    case FIXED_SPEED:
      controller_fixedSpeed(leftOut, rightOut);
      break;
    case PID_CONTROL:
      controller_PID(leftOut, rightOut, velL, velR, dt);
      break;
    case THRESHOLD_PROTECTION:
      controller_thresholdProtection(leftOut, rightOut, d);
      break;
    case PREDICTIVE:
      controller_predictive(leftOut, rightOut, predictedEvent, d);
      break;
    case HYBRID:
      controller_hybrid(leftOut, rightOut, predictedEvent, d, velL, velR, dt);
      break;
  }

  // Safety cutoff regardless of controller mode
  if (d.voltage < VOLTAGE_MIN_V) {
    leftOut = 0;
    rightOut = 0;
  }

#if TEST_MODE
  // No real motors attached — just print what WOULD be sent, plus sensor/controller state
  static unsigned long lastPrint = 0;
  if (now - lastPrint > 250) { // print 4x/sec so it's readable, loop still runs at full rate
    lastPrint = now;
    Serial.print("mode="); Serial.print(currentMode);
    Serial.print(" | current="); Serial.print(d.current, 3);
    Serial.print("A | regressionPred="); Serial.print(lastRegressionPredictedCurrent, 3);
    Serial.print(" | kalmanPred="); Serial.print(kalman.level + kalman.trend * 3, 3);
    Serial.print(" | distFront="); Serial.print(d.distance_mm, 0);
    Serial.print(" | distSide="); Serial.print(d.distance_side_mm, 0);
    Serial.print(" | terrain="); Serial.print(terrainClassName(terrain));
    Serial.print(" | steerBias="); Serial.print(steerBias, 2);
    Serial.print(" | speedFactor="); Serial.print(predictSpeedFactor, 2);
    Serial.print(" | leftPWM="); Serial.print(leftOut, 1);
    Serial.print(" | rightPWM="); Serial.println(rightOut, 1);
  }
#else
  setMotors(leftOut, rightOut);
  logRow(d, currentMode, predictedEvent, actualEvent, terrain);
#endif

  delay(20); // ~50Hz control loop
}
