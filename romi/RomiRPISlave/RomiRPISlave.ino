#include <PololuRPiSlave.h>
#include <Romi32U4.h>

#define I2C_ADDRESS 0x14

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
    uint8_t reset_cmd; 
};

// Robot parameters
const float wheel_diameter = 0.07;
const float encoder_resolution = 1440;
const float distance_per_count = (PI * wheel_diameter) / encoder_resolution;
const float baseline = 0.142;
const float WATCHDOG_TIMEOUT = 500; // ms
unsigned long last_command_time = 0;

// Define velocity ranges and their corresponding PID values
const int NUM_RANGES = 3;  // Can easily add more ranges

const float VELOCITY_THRESHOLDS[NUM_RANGES] = {
    0.17,   // First threshold
    0.24,   // Second threshold
    0.42    // Max velocity
};

const float PID_VALUES[NUM_RANGES][4] = {
    // Kp,   Ki,   Kd,   Kf    (for velocities 0 - 0.17)
    {0.8,   0.04,  0.03,  1.00},  // Original working values
    // For velocities 0.17 - 0.24
    {0.7,   0.02,  0.05,  1.0},  
    // For velocities 0.24 - 0.42
    {0.8,   0.1,  0.05, 0.9}  
};

const float alpha = 0.15;

// Filtered velocity variables
float left_filtered_velocity = 0, right_filtered_velocity = 0;

// I2C communication
PololuRPiSlave<struct Data, 5> slave;

// Romi objects
Romi32U4Motors motors;
Romi32U4Encoders encoders;


// Odometry variables
float x = 0.0, y = 0.0, theta = 0.0;
float left_integral = 0, right_integral = 0;
float left_prev_error = 0, right_prev_error = 0;
int16_t left_encoder_prev = 0, right_encoder_prev = 0;


int16_t velocityPIDFF(float target_velocity, float actual_velocity, float dt, float &integral, float &prev_error) {
    // Find appropriate PID values for current velocity
    int range = 0;
    float abs_velocity = abs(target_velocity);
    
    for(int i = 0; i < NUM_RANGES; i++) {
        if(abs_velocity <= VELOCITY_THRESHOLDS[i]) {
            range = i;
            break;
        }
    }
    
    float Kp = PID_VALUES[range][0];
    float Ki = PID_VALUES[range][1];
    float Kd = PID_VALUES[range][2];
    float Kf = PID_VALUES[range][3];

    float error = target_velocity - actual_velocity;
    // Clear integral when stopping
    if (abs(target_velocity) < 0.01 || 
        (target_velocity * actual_velocity < 0)) {  // If stop command
        integral = 0;
    } else {
        integral = constrain(integral + error * dt, -30, 30);
    }
    float derivative = (error - prev_error) / dt;
    prev_error = error;

    float pid_output = Kp * error + Ki * integral + Kd * derivative;
    float ff_output = Kf * target_velocity;
    
    float total_output = pid_output + ff_output;
    if (!isfinite(total_output)) {
        Serial.println("PID overflow!");
        total_output = 0;
    }
    return velocityToMotorCommand(total_output);
}

int16_t velocityToMotorCommand(float velocity) {
    const float max_velocity = 0.42;    // Max linear
    const int16_t max_power = 300;      
    const int16_t deadzone = 40;
    
    int16_t command = (int16_t)((velocity / max_velocity) * max_power);
    if (abs(command) < deadzone) command = 0;
    if (command > 32767 || command < -32768) {
        Serial.println("Motor command overflow!");
        command = 0;
    }
    return constrain(command, -max_power, max_power);
}

inline float left_desired_velocity(float linear_velocity, float angular_velocity) {
    if (abs(linear_velocity) < 0.01 && abs(angular_velocity) > 0.01) {
        return -angular_velocity * baseline;  // Remove division by 2
    }
    return linear_velocity - (baseline / 2.0) * angular_velocity;
}

inline float right_desired_velocity(float linear_velocity, float angular_velocity) {
    if (abs(linear_velocity) < 0.01 && abs(angular_velocity) > 0.01) {
        return angular_velocity * baseline;  // Remove division by 2
    }
    return linear_velocity + (baseline / 2.0) * angular_velocity;
}

void setup() {
    Serial.begin(115200);
    slave.init(I2C_ADDRESS);
    
    delay(1000);
    
    
}

void loop() {
    slave.updateBuffer();
    // Check for overflow in I2C buffer commands
    const float MAX_VALID_COMMAND = 1.0;  // Maximum reasonable command value
    
    if (slave.buffer.linear_velocity < -MAX_VALID_COMMAND || 
        slave.buffer.linear_velocity > MAX_VALID_COMMAND) {
        Serial.print("Command overflow detected: ");
        Serial.println(slave.buffer.linear_velocity);
        slave.buffer.linear_velocity = 0;
        slave.buffer.angular_velocity = 0;
        motors.setSpeeds(0, 0);
        delay(100);  // Brief pause to avoid rapid oscillations
        return;
    }

    // Update last_command_time when we get valid commands
    if (abs(slave.buffer.linear_velocity) > 0.001 || abs(slave.buffer.angular_velocity) > 0.001) {
        last_command_time = millis();
    }

    if (slave.buffer.reset_cmd == 1) {
        x = 0.0;
        y = 0.0;
        theta = 0.0;
        left_encoder_prev = encoders.getCountsLeft();
        right_encoder_prev = encoders.getCountsRight();
        left_integral = 0;
        right_integral = 0;
        left_prev_error = 0;
        right_prev_error = 0;
        slave.buffer.reset_cmd = 0;  // Clear the command
        Serial.println("Odometry reset by ROS command");
    }

    // Watchdog for command timeout
    unsigned long current_time = millis();
    if (current_time - last_command_time > WATCHDOG_TIMEOUT) {
        motors.setSpeeds(0, 0);
        // Serial.println("Command timeout!");
        return;
    }

    // Validate velocities
    if (isnan(slave.buffer.linear_velocity) || isnan(slave.buffer.angular_velocity) ||
        isinf(slave.buffer.linear_velocity) || isinf(slave.buffer.angular_velocity)) {
        motors.setSpeeds(0, 0);
        Serial.println("Invalid velocity!");
        return;
    }

    static uint64_t lastTime = 0;
    uint64_t now = millis();
    float dt = (now - lastTime) / 1000.0;
    
    if (dt >= 0.02) {  // Ensure minimum 20ms between updates

        Serial.print("CMD Lin,Ang: ");
        Serial.print(slave.buffer.linear_velocity);
        Serial.print(",");
        Serial.println(slave.buffer.angular_velocity);
        

        // Get encoder counts
        int16_t left_encoder = encoders.getCountsLeft();
        int16_t right_encoder = encoders.getCountsRight();
        
        // Calculate deltas
        int16_t left_delta = left_encoder - left_encoder_prev;
        int16_t right_delta = right_encoder - right_encoder_prev;
        
        // Calculate raw velocities
        float left_raw_velocity = left_delta * distance_per_count / dt;
        float right_raw_velocity = right_delta * distance_per_count / dt;
        
        // Apply low-pass filter
        left_filtered_velocity = alpha * left_raw_velocity + (1 - alpha) * left_filtered_velocity;
        right_filtered_velocity = alpha * right_raw_velocity + (1 - alpha) * right_filtered_velocity;
        
        // Calculate desired velocities from ROS commands
        float left_desired = left_desired_velocity(slave.buffer.linear_velocity, slave.buffer.angular_velocity);
        float right_desired = right_desired_velocity(slave.buffer.linear_velocity, slave.buffer.angular_velocity);
        
        // Apply PID with feed-forward
        int16_t left_command = velocityPIDFF(left_desired, left_filtered_velocity, dt, left_integral, left_prev_error);
        int16_t right_command = velocityPIDFF(right_desired, right_filtered_velocity, dt, right_integral, right_prev_error);

        // In loop(), after PID calculations but before setting motor speeds:
        float desired_velocity = (left_desired + right_desired) / 2.0;
        float actual_velocity = (left_filtered_velocity + right_filtered_velocity) / 2.0;
        float command_value = (left_command + right_command) / 2.0;
        
        // Set motor speeds
        motors.setSpeeds(left_command, right_command);
        
        // Calculate distance moved by each wheel since last update
        float left_distance = left_delta * distance_per_count;
        float right_distance = right_delta * distance_per_count;

        // Calculate change in orientation and forward distance
        float delta_theta = (right_distance - left_distance) / baseline;
        float distance = (left_distance + right_distance) / 2.0;

        // Update orientation first
        theta += delta_theta;

        // Normalize theta between -PI and PI
        while (theta > PI) theta -= 2*PI;
        while (theta < -PI) theta += 2*PI;

        // Use average orientation during the move
        float theta_mid = theta - delta_theta/2.0;

        // Update position
        x += distance * cos(theta_mid);
        y += distance * sin(theta_mid);

        // Send POSITION to ROS through I2C buffer
        slave.buffer.x = x;
        slave.buffer.y = y;
        slave.buffer.theta = theta;

        // Debug print
        Serial.print("Position x,y,theta: ");
        Serial.print(x, 3); Serial.print(",");
        Serial.print(y, 3); Serial.print(",");
        Serial.print(theta, 3); Serial.print(" | ");
        
        // Wheel velocities
        Serial.print("Vel L,R: ");
        Serial.print(left_filtered_velocity, 3); Serial.print(",");
        Serial.print(right_filtered_velocity, 3); Serial.print(" | ");
        
        // PID values
        Serial.print("Target L,R: ");
        Serial.print(left_desired, 3); Serial.print(",");
        Serial.println(right_desired, 3);
  
        
        // Update previous values
        left_encoder_prev = left_encoder;
        right_encoder_prev = right_encoder;
        lastTime = now;
        
        // Finalize I2C writes
        slave.finalizeWrites();
    }
}