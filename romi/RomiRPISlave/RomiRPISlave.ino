/* ========== I2C BRIDGE CONFIGURATION ========== */
/* Ensure this exactly matches corresponding uses */

#define TIMEOUT_MS 500.0
#define I2C_ADDRESS 0x14

struct Data {
  int16_t lm_desired;   // Position  0, Length 2
  int16_t rm_desired;   // Position  2, Length 2 

  uint8_t rled;         // Position  4, Length 1
  uint8_t gled;         // Position  5, Length 1
  uint8_t bled;         // Position  6, Length 1

  uint8_t alive;        // Position  7, Length 1

  int64_t lenc_total;   // Position  8, Length 8
  int64_t renc_total;   // Position 16, Length 8

  /* Total Length of 24 bytes */
};

/* ========== I2C BRIDGE CONFIGURATION ========== */

#define LED_PIN 22          // PF1 pin
#define NUM_LEDS 16         // 16 LEDs in total
#define COLOUR_ORDER GRB

#define ENCODER_CLAMP 576                 // Maximum encoder delta per loop, larger counts ignored
#define ENCODER_SMOOTH_DEPTH 128          // Maximum smoothing depth for dynamic encoder calibration 
#define ENCODER_DEFAULT_MULT 0.2500       // Default encoder power multiplier, to calibrate from
#define ENCODER_MINIMUM_MULT 0.1250       // Minimum encoder power multiplier, to calibrate from

#include <math.h>
#include <Servo.h>
#include <Romi32U4.h>
#include <PololuRPiSlave.h>
#include <FastLED.h>

// Constants for robot configuration
const float wheel_diameter = 0.08; // 8 cm in meters
const float encoder_resolution = 1440; // 1440 counts per revolution
const float distance_per_count = (PI * wheel_diameter) / encoder_resolution; // Distance per count in meters
const float max_velocity_left = 0.86; // Maximum velocity in m/s for the left motor
const float max_velocity_right = 0.82; // Maximum velocity in m/s for the right motor

// Exponential smoothing constants
const double ALPHA = 1.0 / (double) ENCODER_SMOOTH_DEPTH;
const double NALPHA = 1.0 - ALPHA;

// Global variables
uint8_t alive;
double timeout;
uint64_t timestamp;

double lencoder_actual_smooth;
double rencoder_actual_smooth;

double lencoder_target_smooth;
double rencoder_target_smooth;

int64_t lencoder;
int64_t rencoder;

PololuRPiSlave<struct Data, 5> slave;
Romi32U4Motors motors;        
Romi32U4Encoders encoders;    
CRGB leds[NUM_LEDS]; 

// PID Controller class
class PIDController {
public:
    PIDController(float Kp, float Ki, float Kd)
        : Kp(Kp), Ki(Ki), Kd(Kd), integral(0), previous_error(0) {}

    float update(float setpoint, float measured_value, float dt) {
        float error = setpoint - measured_value;
        integral += error * dt;
        float derivative = (error - previous_error) / dt;
        float output = Kp * error + Ki * integral + Kd * derivative;
        previous_error = error;
        return output;
    }

private:
    float Kp, Ki, Kd;
    float integral;
    float previous_error;
};

PIDController left_pid(0.69, 0.105, 0.06);
PIDController right_pid(0.69, 0.105, 0.06);

// Function to reset the system state
void reset() {
  slave.buffer.lm_desired = 0;
  slave.buffer.rm_desired = 0;

  slave.buffer.rled = 0;
  slave.buffer.gled = 0;
  slave.buffer.bled = 0;

  lencoder = 0;
  rencoder = 0;

  lencoder_target_smooth = ENCODER_DEFAULT_MULT;
  rencoder_target_smooth = ENCODER_DEFAULT_MULT;

  lencoder_actual_smooth = 1.0;
  rencoder_actual_smooth = 1.0;

  // Ignore values
  int16_t ignored_left = encoders.getCountsAndResetLeft();
  int16_t ignored_right = encoders.getCountsAndResetRight();
}

// Function to check for timeout and reset if necessary
void check_timeout(double dt) {
  if (alive != slave.buffer.alive) {
    timeout = (double) TIMEOUT_MS;
    alive = slave.buffer.alive;
    return;
  }

  timeout -= (dt * 1000.0);
  if (timeout <= 0.0) reset();
}

// Function to calculate the time delta
double delta(uint64_t now) {
  uint64_t delta_us = now - timestamp;
  timestamp = now;
  return (double) delta_us / 1000.0 / 1000.0;
}

// Setup function to initialize the system
void setup() {
  Serial.begin(115200); // Initialize serial communication for debugging
  slave.init(I2C_ADDRESS); // Initialize I2C slave

  // Setup FastLED
  FastLED.addLeds<WS2812, LED_PIN, COLOUR_ORDER>(leds, NUM_LEDS);

  alive = 0x00;
  timeout = (double) TIMEOUT_MS;
  timestamp = micros();

  reset(); // Reset the system state
}

// Main loop function
void loop() {
  double dt = delta(micros()); // Calculate time delta

  // Read and reset encoder counts
  int16_t left_counts = encoders.getCountsAndResetLeft();
  int16_t right_counts = encoders.getCountsAndResetRight();

  // Debug: Print raw encoder counts
  Serial.print("Left Encoder Counts: ");
  Serial.print(left_counts);
  Serial.print(", Right Encoder Counts: ");
  Serial.println(right_counts);

  // Calculate actual velocity from encoder counts in m/s
  float left_actual_velocity = (left_counts * distance_per_count) / dt;
  float right_actual_velocity = (right_counts * distance_per_count) / dt;

  // Debug: Print actual velocities
  Serial.print("Left Actual Velocity (m/s): ");
  Serial.print(left_actual_velocity);
  Serial.print(", Right Actual Velocity (m/s): ");
  Serial.println(right_actual_velocity);

  // Convert desired velocities from I2C buffer to m/s
  float left_desired_velocity = slave.buffer.lm_desired * distance_per_count / dt;
  float right_desired_velocity = slave.buffer.rm_desired * distance_per_count / dt;

  // Debug: Print desired velocities
  Serial.print("Left Desired Velocity (m/s): ");
  Serial.print(left_desired_velocity);
  Serial.print(", Right Desired Velocity (m/s): ");
  Serial.println(right_desired_velocity);

  // Use PID to calculate velocity output
  float left_velocity_output = left_pid.update(left_desired_velocity, left_actual_velocity, dt);
  float right_velocity_output = right_pid.update(right_desired_velocity, right_actual_velocity, dt);

  // Debug: Print PID outputs
  Serial.print("Left PID Output: ");
  Serial.print(left_velocity_output);
  Serial.print(", Right PID Output: ");
  Serial.println(right_velocity_output);

  // Map velocity output to PWM range
  float left_pwm_output = (left_velocity_output / max_velocity_left) * 255.0;
  float right_pwm_output = (right_velocity_output / max_velocity_right) * 255.0;

  // Ensure PWM output is within valid range before sending to motors
  left_pwm_output = constrain(left_pwm_output, -255, 255);
  right_pwm_output = constrain(right_pwm_output, -255, 255);

  // Debug: Print PWM outputs
  Serial.print("Left PWM Output: ");
  Serial.print(left_pwm_output);
  Serial.print(", Right PWM Output: ");
  Serial.println(right_pwm_output);

  motors.setSpeeds(left_pwm_output, right_pwm_output); // Set motor speeds

  // Set and display colors for the LED ring
  for (int i = 0; i < NUM_LEDS; i++) {
    leds[i] = CRGB(slave.buffer.rled, slave.buffer.gled, slave.buffer.bled);
  }
  FastLED.show();

  // Debug: Print LED values
  Serial.print("LED Colors - R: ");
  Serial.print(slave.buffer.rled);
  Serial.print(", G: ");
  Serial.print(slave.buffer.gled);
  Serial.print(", B: ");
  Serial.println(slave.buffer.bled);

  // Update encoder totals in I2C buffer
  lencoder += left_counts;
  rencoder += right_counts;
  slave.buffer.lenc_total = lencoder;
  slave.buffer.renc_total = rencoder;

  // Write latest Data struct to I2C connection
  slave.finalizeWrites();

  delay(100); // Small delay for readability in debugging output
}

