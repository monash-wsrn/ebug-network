
/* ========== I2C BRIDGE CONFIGURATION ========== */
/* Ensure this exactly matches corresponding uses */

#define CONN_TIMEOUT 1000
#define I2C_ADDRESS 0x14
struct Data
{
  int16_t lm_desired;   // Position 0, Length 2
  int16_t rm_desired;   // Position 2, Length 2

  uint8_t rled;         // Position 4, Length 1
  uint8_t gled;         // Position 5, Length 1
  uint8_t bled;         // Position 6, Length 1

  uint8_t alive;        // Position 7, Length 1

  int16_t lm_encoder;   // Position 8, Length 2
  int16_t rm_encoder;   // Position 10, Length 2

  /* Total Length of 11 bytes */
};


/* ========== I2C BRIDGE CONFIGURATION ========== */



#include <Servo.h>
#include <Romi32U4.h>
#include <PololuRPiSlave.h>
#include <FastLED.h>

// Variables for LED connection
#define LED_PIN 22          // PF1 pin
#define NUM_LEDS 16         // 5 LEDs in total but count from 0
#define COLOUR_ORDER GRB


uint64_t time;
uint8_t alive;

PololuRPiSlave<struct Data,5> slave;
Romi32U4Motors motors;
Romi32U4Encoders encoders;
CRGB leds[NUM_LEDS]; 

void setup()
{
  // Set up the slave at I2C address 20.
  slave.init(I2C_ADDRESS);
  alive = 0x00;

  // Setup fast LED
  FastLED.addLeds<WS2812, LED_PIN, COLOUR_ORDER>(leds, NUM_LEDS);
}

void check_timeout()
{
  // If the alive buffer value has been changed, reset timeout 
  if (alive != slave.buffer.alive) {
    time = millis();
    alive = slave.buffer.alive;
    return;
  }

  // Alive buffer value remains unchanged, check timer
  if (millis() - time < CONN_TIMEOUT)
    return;

  // If timed out, set all actionable values to 0
  slave.buffer.lm_desired = 0;
  slave.buffer.rm_desired = 0;

  slave.buffer.rled = 0;
  slave.buffer.gled = 0;
  slave.buffer.bled = 0;
}

void loop()
{
  slave.updateBuffer();
  check_timeout();

  motors.setSpeeds(slave.buffer.lm_desired, slave.buffer.rm_desired);

  // Set and display LED colours
  for (int i = 0; i < NUM_LEDS; i++)
    leds[i] = CRGB(slave.buffer.rled, slave.buffer.gled, slave.buffer.bled);
  FastLED.show();


  slave.buffer.lm_encoder = encoders.getCountsLeft();
  slave.buffer.rm_encoder = encoders.getCountsRight();

  slave.finalizeWrites();
}