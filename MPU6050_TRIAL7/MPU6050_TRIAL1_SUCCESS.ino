/* Edge Inference with MPU6050 and LEDs
 * Runs inference on ESP32, shows classifications on Serial Monitor,
 * and lights LEDs for the highest-probability label
 */

#include <Motion_detection_Mbuch_tu_inferencing.h> //Edge  model
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

Adafruit_MPU6050 mpu;

// LED pins
#define LED1_PIN 17 // Label1
#define LED2_PIN 16 // Label2
#define LED3_PIN 4  // Label3
#define LED4_PIN 2  // Label4

// Constants from Edge model
#define FREQUENCY_HZ EI_CLASSIFIER_FREQUENCY // 100 Hz
#define INTERVAL_MS EI_CLASSIFIER_INTERVAL_MS // 10 ms

// Debug flag
static const bool debug_nn = false;

void setup() {
  // Initialize Serial
  Serial.begin(115200);
  while (!Serial) delay(10);
  Serial.println("Edge Impulse MPU6050 Inference with LEDs");

  // Initialize LEDs
  pinMode(LED1_PIN, OUTPUT);
  pinMode(LED2_PIN, OUTPUT);
  pinMode(LED3_PIN, OUTPUT);
  pinMode(LED4_PIN, OUTPUT);
  // Turn off all LEDs initially
  digitalWrite(LED1_PIN, LOW);
  digitalWrite(LED2_PIN, LOW);
  digitalWrite(LED3_PIN, LOW);
  digitalWrite(LED4_PIN, LOW);

  // Initialize MPU6050
  if (!mpu.begin()) {
    Serial.println("ERROR: Failed to find MPU6050!");
    while (1);
  }
  Serial.println("MPU6050 Found!");

  // Configure MPU6050
  mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  mpu.setHighPassFilter(MPU6050_HIGHPASS_0_63_HZ);
  mpu.setSampleRateDivisor(0);

  // Verify sensor count
  if (EI_CLASSIFIER_RAW_SAMPLES_PER_FRAME != 3) {
    Serial.print("ERR: Model expects ");
    Serial.print(EI_CLASSIFIER_RAW_SAMPLES_PER_FRAME);
    Serial.println(" axes, but MPU6050 provides 3");
    while (1);
  }

  // Verify label count
  if (EI_CLASSIFIER_LABEL_COUNT != 4) {
    Serial.print("ERR: Model has ");
    Serial.print(EI_CLASSIFIER_LABEL_COUNT);
    Serial.println(" labels, but 4 LEDs are configured");
    while (1);
  }

  Serial.println("Setup complete. Starting inference...");
}

void loop() {
  Serial.println("\nStarting inferencing in 2 seconds...");
  delay(2000);
  Serial.println("Sampling...");

  // Allocate buffer for sensor data
  float buffer[EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE] = { 0 };

  // Collect samples
  for (size_t ix = 0; ix < EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE; ix += EI_CLASSIFIER_RAW_SAMPLES_PER_FRAME) {
    int64_t next_tick = (int64_t)micros() + (int64_t)(INTERVAL_MS * 1000);
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    // Use raw accelerometer data (no clamping)
    buffer[ix + 0] = a.acceleration.x;
    buffer[ix + 1] = a.acceleration.y;
    buffer[ix + 2] = a.acceleration.z;

    int64_t wait_time = next_tick - (int64_t)micros();
    if (wait_time > 0) {
      delayMicroseconds(wait_time);
    }
  }

  // Create signal
  signal_t signal;
  int err = numpy::signal_from_buffer(buffer, EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE, &signal);
  if (err != 0) {
    Serial.print("ERR: Signal creation failed (");
    Serial.print(err);
    Serial.println(")");
    return;
  }

  // Run classifier
  ei_impulse_result_t result = { 0 };
  err = run_classifier(&signal, &result, debug_nn);
  if (err != EI_IMPULSE_OK) {
    Serial.print("ERR: Classifier failed (");
    Serial.print(err);
    Serial.println(")");
    return;
  }

  // Print predictions
  Serial.print("Predictions (DSP: ");
  Serial.print(result.timing.dsp);
  Serial.print(" ms, Classification: ");
  Serial.print(result.timing.classification);
  Serial.print(" ms, Anomaly: ");
  Serial.print(result.timing.anomaly);
  Serial.println(" ms):");

  // Find highest-probability label
  float max_value = 0.0;
  size_t max_index = 0;
  for (size_t ix = 0; ix < EI_CLASSIFIER_LABEL_COUNT; ix++) {
    Serial.print("  ");
    Serial.print(result.classification[ix].label);
    Serial.print(": ");
    Serial.println(result.classification[ix].value, 5);

    if (result.classification[ix].value > max_value) {
      max_value = result.classification[ix].value;
      max_index = ix;
    }
  }

  // Control LEDs based on highest-probability label
  digitalWrite(LED1_PIN, max_index == 0 ? HIGH : LOW);
  digitalWrite(LED2_PIN, max_index == 1 ? HIGH : LOW);
  digitalWrite(LED3_PIN, max_index == 2 ? HIGH : LOW);
  digitalWrite(LED4_PIN, max_index == 3 ? HIGH : LOW);

  // Print which LED is on
  Serial.print("LED for ");
  Serial.print(result.classification[max_index].label);
  Serial.println(" is ON");

#if EI_CLASSIFIER_HAS_ANOMALY == 1
  Serial.print("  anomaly score: ");
  Serial.println(result.anomaly, 3);
#endif

  // Delay before next inference
  delay(100);
}