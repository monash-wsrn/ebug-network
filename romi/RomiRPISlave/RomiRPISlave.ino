#include <PololuRPiSlave.h>
#include <Romi32U4.h>

#define I2C_ADDRESS 0x14

// Structure to receive velocity commands
struct Data {
  float linear_velocity;
  float angular_velocity;
};

// I2C communication
PololuRPiSlave<struct Data, 5> slave;

// Romi motor and encoder objects
Romi32U4Motors motors;
Romi32U4Encoders encoders;

// Robot parameters
const float wheel_diameter = 0.08; // 8 cm in meters
const float encoder_resolution = 1440; // 1440 counts per revolution
const float distance_per_count = (PI * wheel_diameter) / encoder_resolution; // Distance per count in meters
const float baseline = 0.142; // Distance between wheels in meters

// Odometry variables
float x = 0.0, y = 0.0, theta = 0.0; // Position (x, y) and orientation (theta)
int16_t left_encoder_prev = 0, right_encoder_prev = 0;

void setup() {
  Serial.begin(115200);
  slave.init(I2C_ADDRESS);
  delay(1000); // Give time to establish the serial connection

  left_encoder_prev = encoders.getCountsLeft();
  right_encoder_prev = encoders.getCountsRight();
}

void loop() {
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
  float left_distance = left_delta * distance_per_count;
  float right_distance = right_delta * distance_per_count;

  // Calculate the change in orientation (theta)
  float delta_theta = (right_distance - left_distance) / baseline;

  // Update the robot's orientation
  theta += delta_theta;

  // Calculate the average distance traveled
  float distance = (left_distance + right_distance) / 2.0;

  // Update the robot's position
  x += distance * cos(theta);
  y += distance * sin(theta);

  // Print out the odometry for debugging
  Serial.print("Position: (");
  Serial.print(x);
  Serial.print(", ");
  Serial.print(y);
  Serial.print(") Orientation: ");
  Serial.println(theta);

  // Set motor speeds based on received velocity commands
  float left_velocity_output = slave.buffer.linear_velocity - (baseline / 2.0) * slave.buffer.angular_velocity;
  float right_velocity_output = slave.buffer.linear_velocity + (baseline / 2.0) * slave.buffer.angular_velocity;

  int left_pwm = map(left_velocity_output * 1000, -255, 255, -255, 255);
  int right_pwm = map(right_velocity_output * 1000, -255, 255, -255, 255);

  motors.setSpeeds(constrain(left_pwm, -255, 255), constrain(right_pwm, -255, 255));

  delay(100); // Small delay for stability
}
