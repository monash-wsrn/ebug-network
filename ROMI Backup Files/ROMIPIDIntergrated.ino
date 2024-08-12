#include <PololuRPiSlave.h>
#include <Romi32U4.h>

// I2C address of Arduino
#define I2C_ADDRESS 0x14

struct Data {
  float linear_velocity;
  float angular_velocity;
};

PololuRPiSlave<struct Data, 5> slave;
Romi32U4Motors motors;
Romi32U4Encoders encoders;

// PID constants
const float Kp = 0.69;
const float Ki = 0.105;
const float Kd = 0.06;

// Robot configuration
const float wheel_diameter = 0.08;  // 8 cm in meters
const float encoder_resolution = 1440;  // 1440 counts per revolution
const float distance_per_count = (PI * wheel_diameter) / encoder_resolution;  // Distance per count in meters
const float max_velocity_left = 0.86;  // Maximum velocity in m/s for the left motor
const float max_velocity_right = 0.82;  // Maximum velocity in m/s for the right motor

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

PIDController left_pid(Kp, Ki, Kd);
PIDController right_pid(Kp, Ki, Kd);

void setup() {
  Serial.begin(115200);
  slave.init(I2C_ADDRESS);
}

void loop() {
  slave.updateBuffer();

  float left_desired_velocity = slave.buffer.linear_velocity - (slave.buffer.angular_velocity * 0.071);  // Adjust based on robot's dimensions
  float right_desired_velocity = slave.buffer.linear_velocity + (slave.buffer.angular_velocity * 0.071);  // Adjust based on robot's dimensions

  static uint64_t lastTime = 0;
  uint64_t now = millis();
  double dt = (now - lastTime) / 1000.0;
  lastTime = now;

  int16_t left_counts = encoders.getCountsAndResetLeft();
  int16_t right_counts = encoders.getCountsAndResetRight();

  float left_actual_velocity = (left_counts * distance_per_count) / dt;
  float right_actual_velocity = (right_counts * distance_per_count) / dt;

  float left_pwm_output = left_pid.update(left_desired_velocity, left_actual_velocity, dt);
  float right_pwm_output = right_pid.update(right_desired_velocity, right_actual_velocity, dt);

  left_pwm_output = constrain((left_pwm_output / max_velocity_left) * 255.0, -255, 255);
  right_pwm_output = constrain((right_pwm_output / max_velocity_right) * 255.0, -255, 255);

  motors.setSpeeds(left_pwm_output, right_pwm_output);

  // Debugging output
  Serial.print("Left Desired: ");
  Serial.print(left_desired_velocity);
  Serial.print(", Actual: ");
  Serial.print(left_actual_velocity);
  Serial.print(", PWM: ");
  Serial.println(left_pwm_output);

  Serial.print("Right Desired: ");
  Serial.print(right_desired_velocity);
  Serial.print(", Actual: ");
  Serial.print(right_actual_velocity);
  Serial.print(", PWM: ");
  Serial.println(right_pwm_output);

  delay(100);
}
