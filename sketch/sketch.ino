#include <Arduino_RouterBridge.h>
#include <array>

// Same eight-float layout as the BMI088 app: ready, target Hz, six axes.
// 0 means unlimited benchmark rate. No sensor, I2C, matrix or serial work.
std::array<float, 8> read_imu_fast() {
  return {1.0f, 0.0f, 0.1f, 0.2f, 9.81f, 0.01f, 0.02f, 0.03f};
}

void setup() {
  Bridge.begin();
  Bridge.provide_safe("read_imu_fast", read_imu_fast);
}

void loop() {
  // Return promptly: provide_safe callbacks are dispatched in the loop thread.
}
