/* ========== I2C BRIDGE CONFIGURATION ========== */
/* Ensure this exactly matches corresponding uses */

#define TIMEOUT_MS 500.0
#define I2C_ADDRESS 0x14
struct Data
{
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


#include <math.h>
#include <Servo.h>
#include <Romi32U4.h>
#include <PololuRPiSlave.h>
#include <FastLED.h>


/* Board    https://www.pololu.com/product/3544 */
/* Chasis   https://www.pololu.com/product/3500 */
/* Wheel    https://www.pololu.com/product/1429 */
/* Motor    https://www.pololu.com/product/1520 */
/* Encoder  https://www.pololu.com/product/3542 */


// Variables for LED connection
#define LED_PIN 22          // PF1 pin
#define NUM_LEDS 16         // 5 LEDs in total but count from 0
#define COLOUR_ORDER GRB

#define ENCODER_CLAMP 576                 // Maximum encoder delta per loop, larger counts ignored
#define ENCODER_SMOOTH_DEPTH 512          // Maximum smoothing depth for dynamic encoder calibration 
#define ENCODER_DEFAULT_MULT 0.2500       // Default encoder power multiplier, to calibrate from
#define ENCODER_MINIMUM_MULT 0.1250       // Minimum encoder power multiplier, to calibrate from

const double ALPHA = 1.0 / (double) ENCODER_SMOOTH_DEPTH;
const double NALPHA = 1.0 - ALPHA;

uint8_t alive;
double timeout;
uint64_t timestamp;

double lencoder_actual_smooth;
double rencoder_actual_smooth;

double lencoder_target_smooth;
double rencoder_target_smooth;

int64_t lencoder;
int64_t rencoder;

PololuRPiSlave<struct Data,5> slave;
Romi32U4Motors motors;        /* https://pololu.github.io/romi-32u4-arduino-library/class_romi32_u4_motors.html   */
Romi32U4Encoders encoders;    /* https://pololu.github.io/romi-32u4-arduino-library/class_romi32_u4_encoders.html */
CRGB leds[NUM_LEDS]; 


void reset()
{
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


void setup()
{
  // Set up the slave at I2C address 20.
  slave.init(I2C_ADDRESS);

  // Setup fast LED
  FastLED.addLeds<WS2812, LED_PIN, COLOUR_ORDER>(leds, NUM_LEDS);
  
  alive = 0x00;
  timeout = (double) TIMEOUT_MS;
  timestamp = micros();

  reset();
}


void check_timeout(double dt)
{
  // If the alive buffer value has been changed, reset timeout 
  if (alive != slave.buffer.alive) {
    timeout = (double) TIMEOUT_MS;
    alive = slave.buffer.alive;
    return;
  }

  // Alive buffer value remains unchanged, check timeout
  timeout -= (dt * 1000.0);
  if (timeout > 0.0)
    return;

  // If timed out, set all actionable values to 0
  reset();
}


double delta(uint64_t now)
{
  uint64_t delta_us = now - timestamp;
  timestamp = now;
  return (double) delta_us / 1000.0 / 1000.0;
}


void loop()
{
  double dt = delta(micros());

  // The actual encoder counts over the period
  // Calculate target v. actual counts discrepancy (exponential moving average)
  // https://stackoverflow.com/a/10990656

  // Calibrate left motor
  int16_t lm_enc_actual = encoders.getCountsAndResetLeft();
  double lm_enc_scaled = fabs((double) lm_enc_actual / dt);
  lencoder_actual_smooth = (ALPHA * lm_enc_scaled) + (NALPHA * lencoder_actual_smooth);

  double lm_enc_target = fabs((double) slave.buffer.lm_desired);
  lencoder_target_smooth = (ALPHA * lm_enc_target) + (NALPHA * lencoder_target_smooth);
  
  double lmultiplier = max(lencoder_target_smooth / lencoder_actual_smooth, ENCODER_MINIMUM_MULT);
  lencoder += lm_enc_actual;

  // Calibrate right motor
  int16_t rm_enc_actual = encoders.getCountsAndResetRight();
  double rm_enc_scaled = fabs((double) rm_enc_actual / dt);
  rencoder_actual_smooth = (ALPHA * rm_enc_scaled) + (NALPHA * rencoder_actual_smooth);

  double rm_enc_target = fabs((double) slave.buffer.rm_desired);
  rencoder_target_smooth = (ALPHA * rm_enc_target) + (NALPHA * rencoder_target_smooth);
  
  double rmultiplier = max(rencoder_target_smooth / rencoder_actual_smooth, ENCODER_MINIMUM_MULT);
  rencoder += rm_enc_actual;

  
  // Read latest Data struct from I2C connection
  slave.updateBuffer();
  check_timeout(dt);
  
  // Scale our new target values to match calculated discrepancy
  int16_t lm_value = (int16_t) ((double) slave.buffer.lm_desired * lmultiplier);
  int16_t rm_value = (int16_t) ((double) slave.buffer.rm_desired * rmultiplier);
  motors.setSpeeds(lm_value, rm_value);

  // Set and display colours for the LED ring
  for (int i = 0; i < NUM_LEDS; i++)
    leds[i] = CRGB(slave.buffer.rled, slave.buffer.gled, slave.buffer.bled);
  FastLED.show();

  slave.buffer.lenc_total = lencoder;
  slave.buffer.renc_total = rencoder;

  // Write latest Data struct to I2C connection
  slave.finalizeWrites();
}