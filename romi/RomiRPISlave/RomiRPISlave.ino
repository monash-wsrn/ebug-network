#include <Romi32U4.h>
#include <PololuRPiSlave.h>
#include <FastLED.h>

// Variables for LED connection
#define LED_PIN 22
#define NUM_LEDS 16
#define COLOUR_ORDER GRB

#define ENCODER_CLAMP 576
#define ENCODER_SMOOTH_DEPTH 128
#define ENCODER_DEFAULT_MULT 0.2500
#define ENCODER_MINIMUM_MULT 0.1250

const double ALPHA = 1.0 / (double)ENCODER_SMOOTH_DEPTH;
const double NALPHA = 1.0 - ALPHA;

uint8_t alive;
double timeout;
uint64_t timestamp;

double lencoder_actual_smooth;
double rencoder_actual_smooth;

double lencoder_target_smooth;
double rencoder_target_smooth;

int16_t lm_enc_actual = 0;
int16_t rm_enc_actual = 0;

int64_t lencoder;
int64_t rencoder;

#define TIMEOUT_MS 500.0
#define I2C_ADDRESS 0x14

struct Data {
    int16_t lm_desired;
    int16_t rm_desired;

    uint8_t rled;
    uint8_t gled;
    uint8_t bled;

    uint8_t alive;

    int64_t lenc_total;
    int64_t renc_total;
};

PololuRPiSlave<struct Data, 5> slave;
Romi32U4Motors motors;
Romi32U4Encoders encoders;
CRGB leds[NUM_LEDS];

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

        // Clamp output to a reasonable range
        if (output > 255) output = 255;
        if (output < -255) output = -255;

        return output;
    }

private:
    float Kp, Ki, Kd;
    float integral;
    float previous_error;
};

// Initialize PID controllers with lower gains
PIDController left_pid(0.1, 0.0, 0.01);
PIDController right_pid(0.1, 0.0, 0.01);

void reset() {
    slave.buffer.lm_desired = 0;
    slave.buffer.rm_desired = 0;

    lencoder = 0;
    rencoder = 0;

    lencoder_target_smooth = ENCODER_DEFAULT_MULT;
    rencoder_target_smooth = ENCODER_DEFAULT_MULT;

    lencoder_actual_smooth = 1.0;
    rencoder_actual_smooth = 1.0;

    int16_t ignored_left = encoders.getCountsAndResetLeft();
    int16_t ignored_right = encoders.getCountsAndResetRight();
}

void setup() {
    Serial.begin(115200);
    slave.init(I2C_ADDRESS);

    FastLED.addLeds<WS2812, LED_PIN, COLOUR_ORDER>(leds, NUM_LEDS);
    
    alive = 0x00;
    timeout = (double)TIMEOUT_MS;
    timestamp = micros();

    reset();
}

void check_timeout(double dt) {
    if (alive != slave.buffer.alive) {
        timeout = (double)TIMEOUT_MS;
        alive = slave.buffer.alive;
        return;
    }

    timeout -= (dt * 1000.0);
    if (timeout > 0.0)
        return;

    reset();
}

double delta(uint64_t now) {
    uint64_t delta_us = now - timestamp;
    timestamp = now;
    double dt = (double) delta_us / 1000.0 / 1000.0;
    if (dt <= 0) dt = 1.0 / 1000.0;  // Ensure dt is never zero or negative
    return dt;
}

int16_t smoothEncoderReading(int16_t new_value, int16_t old_value, float smoothing_factor) {
    return (int16_t)(smoothing_factor * new_value + (1.0 - smoothing_factor) * old_value);
}

void loop() {
    uint64_t now = micros();
    double dt = delta(now);

    lm_enc_actual = smoothEncoderReading(encoders.getCountsAndResetLeft(), lm_enc_actual, 0.1);
    rm_enc_actual = smoothEncoderReading(encoders.getCountsAndResetRight(), rm_enc_actual, 0.1);

    double lm_enc_scaled = fabs((double)lm_enc_actual / dt);
    lencoder_actual_smooth = (ALPHA * lm_enc_scaled) + (NALPHA * lencoder_actual_smooth);

    double lm_enc_target = fabs((double)slave.buffer.lm_desired);
    lencoder_target_smooth = (ALPHA * lm_enc_target) + (NALPHA * lencoder_target_smooth);
    
    double lmultiplier = max(lencoder_target_smooth / lencoder_actual_smooth, ENCODER_MINIMUM_MULT);
    lencoder += lm_enc_actual;

    double rm_enc_scaled = fabs((double)rm_enc_actual / dt);
    rencoder_actual_smooth = (ALPHA * rm_enc_scaled) + (NALPHA * rencoder_actual_smooth);

    double rm_enc_target = fabs((double)slave.buffer.rm_desired);
    rencoder_target_smooth = (ALPHA * rm_enc_target) + (NALPHA * rencoder_target_smooth);
    
    double rmultiplier = max(rencoder_target_smooth / rencoder_actual_smooth, ENCODER_MINIMUM_MULT);
    rencoder += rm_enc_actual;

    slave.updateBuffer();
    check_timeout(dt);

    float lm_output = left_pid.update(slave.buffer.lm_desired, lm_enc_actual, dt);
    float rm_output = right_pid.update(slave.buffer.rm_desired, rm_enc_actual, dt);

    motors.setSpeeds(lm_output, rm_output);

    for (int i = 0; i < NUM_LEDS; i++)
        leds[i] = CRGB(slave.buffer.rled, slave.buffer.gled, slave.buffer.bled);
    FastLED.show();

    lencoder += lm_enc_actual;
    rencoder += rm_enc_actual;

    slave.buffer.lenc_total = lencoder;
    slave.buffer.renc_total = rencoder;

    slave.finalizeWrites();

    Serial.print(slave.buffer.lm_desired);
    Serial.print(",");
    Serial.print(lm_enc_actual);
    Serial.print(",");
    Serial.print(lm_output);
    Serial.print(",");
    Serial.print(slave.buffer.rm_desired);
    Serial.print(",");
    Serial.print(rm_enc_actual);
    Serial.print(",");
    Serial.println(rm_output);
}
