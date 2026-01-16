/*
  ESP32 6-DOF Potentiometer Reader
  Outputs CSV joint angles (degrees) for ROS bridge

  Format:
  j0,j1,j2,j3,j4,j5\n
*/

#include <Arduino.h>

struct Joint {
  int   pin;
  float minDeg;
  float maxDeg;
  float filtered;
};

// --------------------
// Joint configuration
// --------------------
Joint joints[] = {
  {32, -135.0f,  135.0f, 0},  // Joint 0
  {35,  -90.0f,   90.0f, 0},  // Joint 1
  {34,  -90.0f,   90.0f, 0},  // Joint 2
  {33, -135.0f,  135.0f, 0},  // Joint 3
  {36,  -90.0f,   90.0f, 0},  // Joint 4 (VP)
  {39, -135.0f,  135.0f, 0},  // Joint 5 (VN)
};

const int   N_JOINTS = 6;
const float alpha   = 0.15f;    // smoothing (0–1)
const int   hz      = 50;       // output rate
const int   period_ms = 1000 / hz;

// --------------------
// Helper
// --------------------
float adcToDeg(float adc, float minDeg, float maxDeg) {
  float norm = adc / 4095.0f;   // 0–1
  return minDeg + norm * (maxDeg - minDeg);
}

// --------------------
// Arduino lifecycle
// --------------------
void setup() {
  Serial.begin(115200);
  delay(200);

  analogReadResolution(12);        // 0–4095
  analogSetAttenuation(ADC_11db);  // ~0–3.3V

  // Initialize filters
  for (int i = 0; i < N_JOINTS; i++) {
    joints[i].filtered = analogRead(joints[i].pin);
  }
}

void loop() {
  float angles[N_JOINTS];

  // Read + filter + map
  for (int i = 0; i < N_JOINTS; i++) {
    int raw = analogRead(joints[i].pin);

    joints[i].filtered =
      (1.0f - alpha) * joints[i].filtered + alpha * raw;

    angles[i] = adcToDeg(
      joints[i].filtered,
      joints[i].minDeg,
      joints[i].maxDeg
    );
  }

  // CSV output (ROS-friendly)
  Serial.printf(
    "%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\n",
    angles[0], angles[1], angles[2],
    angles[3], angles[4], angles[5]
  );

  delay(period_ms);
}
