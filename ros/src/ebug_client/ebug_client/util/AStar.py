# Copyright Pololu Corporation.  For more information, see https://www.pololu.com/

# gyroscope code reference
# 1. https://github.com/DarkSparkAg/MinIMU-9-v5/blob/master/MinIMU_v5_pi.py
# 2. http://rpi-data.blogspot.com/2016/09/programming-i2c-bus-in-python.html

import smbus
import struct
import time
import math

class AStar:
  def __init__(self):
    self.bus = smbus.SMBus(1)

    #activate gyro
    self.bus.write_byte_data(0x6b, 0x16, 0x38)
    self.bus.write_byte_data(0x6b, 0x11, 0x60) # Initialise to 245dps, with sensitivity gain of 0.00875
    self.G_GAIN = 0.00875


    self.time_prev = time.time()

  def read_unpack(self, address, size, format):
    # Ideally we could do this:
    #    byte_list = self.bus.read_i2c_block_data(20, address, size)
    # But the AVR's TWI module can't handle a quick write->read transition,
    # since the STOP interrupt will occasionally happen after the START
    # condition, and the TWI module is disabled until the interrupt can
    # be processed.
    #
    # A delay of 0.0001 (100 us) after each write is enough to account
    # for the worst-case situation in our example code.

    self.bus.write_byte(20, address)
    time.sleep(0.0002)
    byte_list = [self.bus.read_byte(20) for _ in range(size)]
    return struct.unpack(format, bytes(byte_list))

  def write_pack(self, address, format, *data):
    data_array = list(struct.pack(format, *data))
    self.bus.write_i2c_block_data(20, address, data_array)
    time.sleep(0.0002)

  #def leds(self, red, yellow, green):
  #  self.write_pack(0, 'BBB', red, yellow, green)

  def leds(self, red, green, blue):
    self.write_pack(0, 'ccc', red, green, blue)

  def play_notes(self, notes):
    self.write_pack(24, 'B14s', 1, notes.encode("ascii"))

  def motors(self, left, right):
    self.write_pack(6, 'hh', left, right)

  def read_buttons(self):
    return self.read_unpack(3, 3, "???")

  def read_battery_millivolts(self):
    return self.read_unpack(10, 2, "H")

  def read_analog(self):
    return self.read_unpack(12, 12, "HHHHHH")

  def read_encoders(self):
    return self.read_unpack(39, 4, 'hh')

  def test_read8(self):
    self.read_unpack(0, 8, 'cccccccc')

  def test_write8(self):
    self.bus.write_i2c_block_data(20, 0, [0,0,0,0,0,0,0,0])
    time.sleep(0.0001)

  def read_gyroscope(self):

    # read back the gyro values
    gx = self.byteToNumber(self.bus.read_byte_data(0x6b, 0x22), self.bus.read_byte_data(0x6b, 0x23))
    gy = self.byteToNumber(self.bus.read_byte_data(0x6b, 0x24), self.bus.read_byte_data(0x6b, 0x25))
    gz = self.byteToNumber(self.bus.read_byte_data(0x6b, 0x26), self.bus.read_byte_data(0x6b, 0x27))

    ## uses some constant offset to correct the error when measured statically
    wx = math.radians(gx  * self.G_GAIN - 4)
    wy = math.radians(gy  * self.G_GAIN + 8)
    wz = math.radians(gz  * self.G_GAIN + 5)

    return (wx, wy, wz) # in radian per seconds

  def byteToNumber(self, val_Low, val_Hi):
    """Combines Hi and Low 8-bit values to a 16-bit two's complement and
	  converts to decimal"""

    number = 256 * val_Hi + val_Low #2^8 = 256
    if number >= 32768: #2^7 = 32768
      number= number - 65536 #For two's complement
    return number

