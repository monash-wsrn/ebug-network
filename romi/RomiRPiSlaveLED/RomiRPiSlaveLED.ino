/* ========== I2C BRIDGE CONFIGURATION ========== */
/* Ensure this exactly matches corresponding uses */

#define TIMEOUT_MS 5000.0
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


#define DEFAULT_MULTIPLIER 0.5            // Default Encoder-Counts to Motor-Speed conversion factor
#define MAX_MULTIPLIER_DEVIATION 0.10     // How much each motor calibration can deviate from the default multiplier 
#define ENCODER_SMOTHING_DEPTH 16384      // Depth of exponential rolling mean for individual motor calibartion


uint8_t alive;
double timeout;
uint64_t timestamp;

double lmultiplier;
double rmultiplier;

int64_t lencoder;
int64_t rencoder;

const double alpha = 1.0 / ENCODER_SMOTHING_DEPTH;
const double nalpha = 1.0 - alpha; 


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
  
  lmultiplier = 1.0;
  rmultiplier = 1.0;

  // Ignore values
  int16_t ignored_left = encoders.getCountsAndResetLeft();
  int16_t ignored_right = encoders.getCountsAndResetRight();
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

  // The actual encoder counts over the period
  int16_t lm_enc_actual = encoders.getCountsAndResetLeft();
  int16_t rm_enc_actual = encoders.getCountsAndResetRight();

  // The target encoder counts over the period
  double lm_enc_target = (double) slave.buffer.lm_desired * dt;
  double rm_enc_target = (double) slave.buffer.rm_desired * dt;

  // Calculate target v. actual counts discrepancy (exponential moving average)
  // https://stackoverflow.com/a/10990656
  if (lm_enc_actual != 0 && lm_enc_target != 0)
    lmultiplier = ((lm_enc_target / (double) lm_enc_actual) * alpha) + (nalpha * lmultiplier);
  
  if (rm_enc_actual != 0 && rm_enc_target != 0)
    rmultiplier = ((rm_enc_target / (double) rm_enc_actual) * alpha) + (nalpha * rmultiplier);
  
  double clmultiplier = (DEFAULT_MULTIPLIER * (1.0 - MAX_MULTIPLIER_DEVIATION)) + (lmultiplier * MAX_MULTIPLIER_DEVIATION);
  double crmultiplier = (DEFAULT_MULTIPLIER * (1.0 - MAX_MULTIPLIER_DEVIATION)) + (rmultiplier * MAX_MULTIPLIER_DEVIATION);

  // Read latest Data struct from I2C connection
  slave.updateBuffer();
  check_timeout(dt);
  
  // Scale our new target values to match calculated discrepancy
  int16_t lm_value = (int16_t) ((double) slave.buffer.lm_desired * clmultiplier);
  int16_t rm_value = (int16_t) ((double) slave.buffer.rm_desired * crmultiplier);
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