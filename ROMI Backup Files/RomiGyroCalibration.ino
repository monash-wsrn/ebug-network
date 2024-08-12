#include <Wire.h>
#include <LSM6.h>

LSM6 imu;  // Assuming you're using the LSM6DS33 sensor

float gyro_offset_x = 0.0;
float gyro_offset_y = 0.0;
float gyro_offset_z = 0.0;

void setup() {
  Serial.begin(115200);
  Wire.begin();
  
  if (!imu.init()) {
    Serial.println("Failed to detect and initialize IMU!");
    while (1);
  }

  imu.enableDefault();
  calibrateGyro();  // Calibrate the gyro at startup
}

void calibrateGyro() {
  int num_samples = 1000;
  float gx, gy, gz;

  Serial.println("Calibrating gyroscope...");

  for (int i = 0; i < num_samples; i++) {
    imu.readGyro();

    gx += imu.g.x;
    gy += imu.g.y;
    gz += imu.g.z;

    delay(10);
  }

  // Calculate the average offset
  gyro_offset_x = gx / num_samples;
  gyro_offset_y = gy / num_samples;
  gyro_offset_z = gz / num_samples;

  Serial.print("Gyro offsets - X: ");
  Serial.print(gyro_offset_x);
  Serial.print(" Y: ");
  Serial.print(gyro_offset_y);
  Serial.print(" Z: ");
  Serial.println(gyro_offset_z);
}

void loop() {
  imu.readGyro();

  // Subtract the offsets from the raw data
  float gx = imu.g.x - gyro_offset_x;
  float gy = imu.g.y - gyro_offset_y;
  float gz = imu.g.z - gyro_offset_z;

  // Print out the gyro data
  Serial.print("Gyro X: ");
  Serial.print(gx);
  Serial.print(" rad/s, Y: ");
  Serial.print(gy);
  Serial.print(" rad/s, Z: ");
  Serial.println(gz);
  
  delay(100);
}
