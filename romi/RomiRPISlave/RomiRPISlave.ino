#include <PololuRPiSlave.h>
#include <Romi32U4.h>
#include <FastLED.h>

#define I2C_ADDRESS 0x14
#define LED_PIN 22
#define NUM_LEDS 16
#define COLOUR_ORDER GRB

#define LSM6_ADDRESS 0x6B

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

// LSM6 variables
float gyro_x, gyro_y, gyro_z;
float gyro_offset_x = 0.0, gyro_offset_y = 0.0, gyro_offset_z = 0.0;
const float gyro_sensitivity = 8.75f / 1000.0f; // FS=250dps

void writeI2CRegister(uint8_t addr, uint8_t reg, uint8_t val) {
  TWCR = 0;
  TWSR = 0;
  TWBR = ((F_CPU / 400000L) - 16) / 2;

  TWCR = (1<<TWINT) | (1<<TWSTA) | (1<<TWEN); // Send START condition
  while (!(TWCR & (1<<TWINT)));

  TWDR = addr << 1; // Send slave address with write bit
  TWCR = (1<<TWINT) | (1<<TWEN);
  while (!(TWCR & (1<<TWINT)));

  TWDR = reg; // Send register address
  TWCR = (1<<TWINT) | (1<<TWEN);
  while (!(TWCR & (1<<TWINT)));

  TWDR = val; // Send value to write
  TWCR = (1<<TWINT) | (1<<TWEN);
  while (!(TWCR & (1<<TWINT)));

  TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWSTO); // Send STOP condition
}

void readI2CRegisters(uint8_t addr, uint8_t reg, uint8_t *buffer, uint8_t num) {
  TWCR = 0;
  TWSR = 0;
  TWBR = ((F_CPU / 400000L) - 16) / 2;

  TWCR = (1<<TWINT) | (1<<TWSTA) | (1<<TWEN); // Send START condition
  while (!(TWCR & (1<<TWINT)));

  TWDR = addr << 1; // Send slave address with write bit
  TWCR = (1<<TWINT) | (1<<TWEN);
  while (!(TWCR & (1<<TWINT)));

  TWDR = reg; // Send register address
  TWCR = (1<<TWINT) | (1<<TWEN);
  while (!(TWCR & (1<<TWINT)));

  TWCR = (1<<TWINT) | (1<<TWSTA) | (1<<TWEN); // Send REPEATED START condition
  while (!(TWCR & (1<<TWINT)));

  TWDR = (addr << 1) | 0x01; // Send slave address with read bit
  TWCR = (1<<TWINT) | (1<<TWEN);
  while (!(TWCR & (1<<TWINT)));

  for (uint8_t i = 0; i < num - 1; i++) {
    TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWEA); // Read with ACK
    while (!(TWCR & (1<<TWINT)));
    buffer[i] = TWDR;
  }

  TWCR = (1<<TWINT) | (1<<TWEN); // Read last byte with NACK
  while (!(TWCR & (1<<TWINT)));
  buffer[num - 1] = TWDR;

  TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWSTO); // Send STOP condition
}

void initLSM6() {
  writeI2CRegister(LSM6_ADDRESS, 0x10, 0x38); // CTRL1_XL: Enable accelerometer, 416 Hz, +/- 2g
  writeI2CRegister(LSM6_ADDRESS, 0x11, 0x38); // CTRL2_G: Enable gyroscope, 416 Hz, 250 dps
}

void readGyro() {
  uint8_t buffer[6];
  readI2CRegisters(LSM6_ADDRESS, 0x22, buffer, 6);

  int16_t gx = (int16_t)(buffer[1] << 8 | buffer[0]);
  int16_t gy = (int16_t)(buffer[3] << 8 | buffer[2]);
  int16_t gz = (int16_t)(buffer[5] << 8 | buffer[4]);

  gyro_x = (gx - gyro_offset_x) * gyro_sensitivity;
  gyro_y = (gy - gyro_offset_y) * gyro_sensitivity;
  gyro_z = (gz - gyro_offset_z) * gyro_sensitivity;
}

void calibrateGyro() {
  const int num_samples = 1000;
  int32_t gx_sum = 0, gy_sum = 0, gz_sum = 0;

  Serial.println("Calibrating gyroscope...");
  Serial.println("Please keep the robot still.");

  for (int i = 0; i < num_samples; i++) {
    uint8_t buffer[6];
    readI2CRegisters(LSM6_ADDRESS, 0x22, buffer, 6);

    int16_t gx = (int16_t)(buffer[1] << 8 | buffer[0]);
    int16_t gy = (int16_t)(buffer[3] << 8 | buffer[2]);
    int16_t gz = (int16_t)(buffer[5] << 8 | buffer[4]);

    gx_sum += gx;
    gy_sum += gy;
    gz_sum += gz;

    delay(5);  // Short delay between readings
  }

  gyro_offset_x = (float)gx_sum / num_samples;
  gyro_offset_y = (float)gy_sum / num_samples;
  gyro_offset_z = (float)gz_sum / num_samples;

  Serial.println("Gyroscope calibration complete.");
  Serial.print("Gyro offsets - X: ");
  Serial.print(gyro_offset_x);
  Serial.print(" Y: ");
  Serial.print(gyro_offset_y);
  Serial.print(" Z: ");
  Serial.println(gyro_offset_z);
}

void setup() {
  Serial.begin(115200);
  while (!Serial) {
    ; // Wait for serial port to connect
  }
  
  slave.init(I2C_ADDRESS);
  
  left_encoder_prev = encoders.getCountsLeft();
  right_encoder_prev = encoders.getCountsRight();

  FastLED.addLeds<WS2812, LED_PIN, COLOUR_ORDER>(leds, NUM_LEDS);

  initLSM6();
  calibrateGyro();

  // Wait for a moment after calibration
  delay(1000);
}

void loop() {
  slave.updateBuffer();

  static uint32_t lastTime = 0;
  uint32_t now = millis();
  float dt = (now - lastTime) / 1000.0;
  lastTime = now;

  int16_t left_encoder = encoders.getCountsLeft();
  int16_t right_encoder = encoders.getCountsRight();

  int16_t left_delta = left_encoder - left_encoder_prev;
  int16_t right_delta = right_encoder - right_encoder_prev;

  left_encoder_prev = left_encoder;
  right_encoder_prev = right_encoder;

  float left_distance = left_delta * distance_per_count;
  float right_distance = right_delta * distance_per_count;

  float delta_theta_encoders = (right_distance - left_distance) / baseline;

  readGyro();

  // Sensor fusion
  float gyro_weight = 0.98;
  float delta_theta = gyro_weight * gyro_z * dt + (1 - gyro_weight) * delta_theta_encoders;

  theta += delta_theta;

  float distance = (left_distance + right_distance) / 2.0;

  x += distance * cos(theta);
  y += distance * sin(theta);

  // Calculate desired velocities from received commands
  float left_desired_velocity = slave.buffer.linear_velocity - (baseline / 2.0) * slave.buffer.angular_velocity;
  float right_desired_velocity = slave.buffer.linear_velocity + (baseline / 2.0) * slave.buffer.angular_velocity;

  // Calculate actual velocities from encoder counts
  float left_actual_velocity = left_distance / dt;
  float right_actual_velocity = right_distance / dt;

  // Use PID to calculate velocity output
  float left_velocity_output = left_pid.update(left_desired_velocity, left_actual_velocity, dt);
  float right_velocity_output = right_pid.update(right_desired_velocity, right_actual_velocity, dt);

  // Map velocity output to PWM range
  int left_pwm = (left_velocity_output / max_velocity_left) * 255.0;
  int right_pwm = (right_velocity_output / max_velocity_right) * 255.0;

  // Set motor speeds
  motors.setSpeeds(constrain(left_pwm, -255, 255), constrain(right_pwm, -255, 255));

  // Set LED colors
  for (int i = 0; i < NUM_LEDS; i++) {
    leds[i] = CRGB(slave.buffer.rled, slave.buffer.gled, slave.buffer.bled);
  }
  FastLED.show();

  // Update the odometry values in the I2C buffer
  slave.buffer.x = x;
  slave.buffer.y = y;
  slave.buffer.theta = theta;

  // Increment the alive counter
  slave.buffer.alive = (slave.buffer.alive + 1) % 256;

  slave.finalizeWrites();

  Serial.print("Position: (");
  Serial.print(x);
  Serial.print(", ");
  Serial.print(y);
  Serial.print(") Orientation: ");
  Serial.print(theta);
  Serial.print(" Gyro Z: ");
  Serial.print(gyro_z);
  Serial.println(" rad/s");

  delay(10);
}