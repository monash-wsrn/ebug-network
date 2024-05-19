
/* ========== I2C BRIDGE CONFIGURATION ========== */
/* Ensure this exactly matches corresponding uses */

#define I2C_ADDRESS 0x14
struct Data
{
  int16_t leftMotor;    // Position 0, Length 2
  int16_t rightMotor;   // Position 2, Length 2

  uint8_t rled;         // Position 4, Length 1
  uint8_t gled;         // Position 5, Length 1
  uint8_t bled;         // Position 6, Length 1

  int16_t leftEncoder;  // Position 7, Length 2
  int16_t rightEncoder; // Position 9, Length 2

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

PololuRPiSlave<struct Data,5> slave;

Romi32U4Motors motors;
Romi32U4Encoders encoders;
CRGB leds[NUM_LEDS]; 

void setup()
{
  // Set up the slave at I2C address 20.
  slave.init(I2C_ADDRESS);

  // Setup fast LED
  FastLED.addLeds<WS2812, LED_PIN, COLOUR_ORDER>(leds, NUM_LEDS);
}

void loop()
{
  // Call updateBuffer() before using the buffer, to get the latest
  // data including recent master writes.
  slave.updateBuffer();

  motors.setSpeeds(slave.buffer.leftMotor, slave.buffer.rightMotor);

  // Set and display LED colours
  for (int i = 0; i < NUM_LEDS; i++)
    leds[i] = CRGB(slave.buffer.rled, slave.buffer.gled, slave.buffer.bled);
  FastLED.show();



  slave.buffer.leftEncoder = encoders.getCountsLeft();
  slave.buffer.rightEncoder = encoders.getCountsRight();

  // When you are done WRITING, call finalizeWrites() to make modified
  // data available to I2C master.
  slave.finalizeWrites();
}
