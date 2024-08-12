#include <Romi32U4.h>

Romi32U4Motors motors;
Romi32U4Encoders encoders;

const float wheel_diameter = 0.08; // 8 cm in meters
const float encoder_resolution = 1440; // 1440 counts per revolution
const float distance_per_count = (PI * wheel_diameter) / encoder_resolution; // Distance per count in meters
const float max_velocity_left = 0.86; // Maximum velocity in m/s for the left motor
const float max_velocity_right = 0.82; // Maximum velocity in m/s for the right motor

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

void setup() {
    Serial.begin(115200);
    delay(10000); // Give time to open serial monitor
}

void loop() {
    static uint64_t lastTime = 0;
    uint64_t now = millis();
    double dt = (now - lastTime) / 1000.0;
    lastTime = now;

    // Set your desired velocity in m/s
    float left_desired_velocity = 0; // Desired speed in m/s
    float right_desired_velocity = 0; // Desired speed in m/s

    int16_t left_counts = encoders.getCountsAndResetLeft();
    int16_t right_counts = encoders.getCountsAndResetRight();

    // Calculate actual velocity from encoder counts in m/s
    float left_actual_velocity = (left_counts * distance_per_count) / dt;
    float right_actual_velocity = (right_counts * distance_per_count) / dt;

    // Use PID to calculate velocity output
    float left_velocity_output = left_pid.update(left_desired_velocity, left_actual_velocity, dt);
    float right_velocity_output = right_pid.update(right_desired_velocity, right_actual_velocity, dt);

    // Map velocity output to PWM range
    float left_pwm_output = (left_velocity_output / max_velocity_left) * 255.0;
    float right_pwm_output = (right_velocity_output / max_velocity_right) * 255.0;

    // Ensure PWM output is within valid range before sending to motors
    left_pwm_output = constrain(left_pwm_output, -255, 255);
    right_pwm_output = constrain(right_pwm_output, -255, 255);

    motors.setSpeeds(left_pwm_output, right_pwm_output);

    Serial.print(left_desired_velocity);
    Serial.print(",");
    Serial.print(left_actual_velocity);
    Serial.print(",");
    Serial.print(left_pwm_output);
    Serial.print(",");
    Serial.print(right_desired_velocity);
    Serial.print(",");
    Serial.print(right_actual_velocity);
    Serial.print(",");
    Serial.println(right_pwm_output);

    delay(100);
}
