#include <PololuRPiSlave.h>
#include <Romi32U4.h>
#include <FastLED.h>

#define I2C_ADDRESS 0x14
#define LED_PIN 22
#define NUM_LEDS 16
#define COLOUR_ORDER GRB

// Structure to receive velocity commands and LED color, and send odometry data
struct Data {
float linear_velocity;
float angular_velocity;
uint8_t rled;
uint8_t gled;
uint8_t bled;
uint8_t alive;
float x;
float y;
float theta;
};

// Robot parameters
const float wheel_diameter = 0.08;
const float encoder_resolution = 1440;
const float distance_per_count = (PI * wheel_diameter) / encoder_resolution;
const float baseline = 0.142;
const float max_velocity_left = 0.86;
const float max_velocity_right = 0.82;

// Inline functions for distance calculations
inline float left_distance(int16_t left_delta) {
return left_delta * distance_per_count;
}
inline float right_distance(int16_t right_delta) {
return right_delta * distance_per_count;
}

// Inline functions for desired velocity calculations
inline float left_desired_velocity(float linear_velocity, float angular_velocity) {
return linear_velocity - (baseline / 2.0) * angular_velocity;
}
inline float right_desired_velocity(float linear_velocity, float angular_velocity) {
return linear_velocity + (baseline / 2.0) * angular_velocity;
}

// PID controller class
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

// I2C communication
PololuRPiSlave<struct Data, 5> slave;

// Romi motor and encoder objects
Romi32U4Motors motors;
Romi32U4Encoders encoders;

// PID controllers for left and right motors
PIDController left_pid(0.69, 0.105, 0.06);
PIDController right_pid(0.69, 0.105, 0.06);

// LED ring
CRGB leds[NUM_LEDS];

// Odometry variables
float x = 0.0, y = 0.0, theta = 0.0;
int16_t left_encoder_prev = 0, right_encoder_prev = 0;

void setup() {
// Initialize serial communication
Serial.begin(115200);

// Initialize I2C slave
slave.init(I2C_ADDRESS);

// Give time to establish the serial connection
delay(1000);

// Get initial encoder counts
left_encoder_prev = encoders.getCountsLeft();
right_encoder_prev = encoders.getCountsRight();

// Setup FastLED library
FastLED.addLeds<WS2812, LED_PIN, COLOUR_ORDER>(leds, NUM_LEDS);
}

void loop() {
// Update the I2C buffer
slave.updateBuffer();

// Calculate time step
static uint64_t lastTime = 0;
uint64_t now = millis();
double dt = (now - lastTime) / 1000.0;
lastTime = now;

// Get the current encoder counts
int16_t left_encoder = encoders.getCountsLeft();
int16_t right_encoder = encoders.getCountsRight();

// Calculate the difference in encoder counts
int16_t left_delta = left_encoder - left_encoder_prev;
int16_t right_delta = right_encoder - right_encoder_prev;

// Update the previous encoder counts
left_encoder_prev = left_encoder;
right_encoder_prev = right_encoder;

// Calculate the distance each wheel has traveled
float left_distance_val = left_distance(left_delta);
float right_distance_val = right_distance(right_delta);

// Calculate the change in orientation (theta)
float delta_theta = (right_distance_val - left_distance_val) / baseline;
theta += delta_theta;

// Calculate the average distance traveled
float distance = (left_distance_val + right_distance_val) / 2.0;

// Update the robot's position
x += distance * cos(theta);
y += distance * sin(theta);

// Calculate desired velocities from received commands
float left_desired_velocity_val = left_desired_velocity(slave.buffer.linear_velocity, slave.buffer.angular_velocity);
float right_desired_velocity_val = right_desired_velocity(slave.buffer.linear_velocity, slave.buffer.angular_velocity);

// Calculate actual velocities from encoder counts
float left_actual_velocity = left_distance_val / dt;
float right_actual_velocity = right_distance_val / dt;

// Use PID to calculate velocity output
float left_velocity_output = left_pid.update(left_desired_velocity_val, left_actual_velocity, dt);
float right_velocity_output = right_pid.update(right_desired_velocity_val, right_actual_velocity, dt);

// Map velocity output to PWM range
int left_pwm = (left_velocity_output / max_velocity_left) * 255.0;
int right_pwm = (right_velocity_output / max_velocity_right) * 255.0;

// Set motor speeds
motors.setSpeeds(constrain(left_pwm, -255, 255), constrain(right_pwm, -255, 255));

// Set and display colors for the LED ring
for (int i = 0; i < NUM_LEDS; i++)
  leds[i] = CRGB(slave.buffer.rled, slave.buffer.gled, slave.buffer.bled);
FastLED.show();

// Update the odometry values in the I2C buffer
slave.buffer.x = x;
slave.buffer.y = y;
slave.buffer.theta = theta;

// Finalize the writes to the I2C buffer
slave.finalizeWrites();

// Print out the odometry for debugging
Serial.print("Position: (");
Serial.print(x);
Serial.print(", ");
Serial.print(y);
Serial.print(") Orientation: ");
Serial.println(theta);


// Small delay for stability
delay(100);
}