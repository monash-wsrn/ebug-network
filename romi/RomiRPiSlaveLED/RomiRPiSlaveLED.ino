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

#define ENCODER_SMOTHING_DEPTH 10

uint8_t alive;
double timeout;
uint64_t timestamp;

uint8_t multindex;

double lmultvals[ENCODER_SMOTHING_DEPTH];
double rmultvals[ENCODER_SMOTHING_DEPTH];

double lmultagg;
double rmultagg;

int64_t lencoder;
int64_t rencoder;

PololuRPiSlave<struct Data,5> slave;
Romi32U4Motors motors;        /* https://pololu.github.io/romi-32u4-arduino-library/class_romi32_u4_motors.html   */
Romi32U4Encoders encoders;    /* https://pololu.github.io/romi-32u4-arduino-library/class_romi32_u4_encoders.html */
CRGB leds[NUM_LEDS]; 

void setup()
{
  // Set up the slave at I2C address 20.
  slave.init(I2C_ADDRESS);

  // Setup fast LED
  FastLED.addLeds<WS2812, LED_PIN, COLOUR_ORDER>(leds, NUM_LEDS);
  
  alive = 0x00;
  timeout = (double) TIMEOUT_MS;
  timestamp = micros();

  lencoder = 0;
  rencoder = 0;

  // Ignore values
  int16_t ignored_left = encoders.getCountsAndResetLeft();
  int16_t ignored_right = encoders.getCountsAndResetRight();

  multindex = 0;
  for (int i = 0; i < ENCODER_SMOTHING_DEPTH; i++)
  {
    lmultvals[i] = 1.0;
    lmultagg += 1.0;

    rmultvals[i] = 1.0;
    rmultagg += 1.0;
  }
}


void aggregate_encoder_multiplier(const double lm_enc_actual, const double rm_enc_actual, const double dt)
{
  lmultagg -= lmultvals[multindex];
  rmultagg -= rmultvals[multindex];

  // The target encoder counts over the period
  double lm_enc_target = (double) slave.buffer.lm_desired * dt;
  double rm_enc_target = (double) slave.buffer.rm_desired * dt;

  // Calculate target v. actual counts discrepancy
  if (lm_enc_actual != 0)
    lmultvals[multindex] = lm_enc_target / (double) lm_enc_actual;
  else
    lmultvals[multindex] = lmultagg / ENCODER_SMOTHING_DEPTH;
  
  if (rm_enc_actual != 0)
    rmultvals[multindex] = rm_enc_target / (double) rm_enc_actual;
  else
    rmultvals[multindex] = rmultagg / ENCODER_SMOTHING_DEPTH;


  lmultagg += lmultvals[multindex];
  rmultagg += rmultvals[multindex];
  
  multindex = (multindex + 1) % ENCODER_SMOTHING_DEPTH;
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
  slave.buffer.lm_desired = 0;
  slave.buffer.rm_desired = 0;

  slave.buffer.rled = 0;
  slave.buffer.gled = 0;
  slave.buffer.bled = 0;
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
  check_timeout(dt);

  // The actual encoder counts over the period
  int16_t lm_enc_actual = encoders.getCountsAndResetLeft();
  int16_t rm_enc_actual = encoders.getCountsAndResetRight();

  // Smooth over the encoder multipliers
  aggregate_encoder_multiplier(lm_enc_actual, rm_enc_actual, dt);

  // Read latest Data struct from I2C connection
  slave.updateBuffer();
  
  // Scale our new target values to match calculated discrepancy
  int16_t lm_value = (int16_t) ((double) slave.buffer.lm_desired * (lmultagg / ENCODER_SMOTHING_DEPTH));
  int16_t rm_value = (int16_t) ((double) slave.buffer.rm_desired * (rmultagg / ENCODER_SMOTHING_DEPTH));
  motors.setSpeeds(lm_value, rm_value);

  // Set and display colours for the LED ring
  for (int i = 0; i < NUM_LEDS; i++)
    leds[i] = CRGB(slave.buffer.rled, slave.buffer.gled, slave.buffer.bled);
  FastLED.show();

  
  lencoder += (int64_t) lm_enc_actual;
  rencoder += (int64_t) rm_enc_actual;

  slave.buffer.lenc_total = lencoder;
  slave.buffer.renc_total = rencoder;

  // Write latest Data struct to I2C connection
  slave.finalizeWrites();
}