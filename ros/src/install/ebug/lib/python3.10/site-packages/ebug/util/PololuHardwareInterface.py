# Copyright Pololu Corporation.  For more information, see https://www.pololu.com/

# gyroscope code reference
# 1. https://github.com/DarkSparkAg/MinIMU-9-v5/blob/master/MinIMU_v5_pi.py
# 2. http://rpi-data.blogspot.com/2016/09/programming-i2c-bus-in-python.html

import smbus
import struct
import time
import math

class PololuHardwareInterface:
  def __init__(self, retry_max):
    self.retry_max = retry_max
    self.bus = smbus.SMBus(1)

    # https://docs.python.org/3/library/struct.html#format-characters

    #activate gyro
    self.bus.write_byte_data(0x6b, 0x16, 0x38)
    self.bus.write_byte_data(0x6b, 0x11, 0x60) # Initialise to 245dps, with sensitivity gain of 0.00875
    self.G_GAIN = 0.00875

    self.time_prev = time.time()

    self.alive = 0
    self.write_alive()


  def read_unpack(self, address, size, format):
    self.bus.write_byte(20, address)
    time.sleep(0.0002)  # Sleep for 0.2 millis

    byte_list = [self.bus.read_byte(20) for _ in range(size)]
    return struct.unpack(format, bytes(byte_list))

  def write_pack(self, address, format, *data):
    data_array = list(struct.pack(format, *data))

    self.bus.write_i2c_block_data(20, address, data_array)
    time.sleep(0.0002)  # Sleep for 0.2 millis

  def read_gryo_internal(self):
    gx = self.byteToNumber(self.bus.read_byte_data(0x6b, 0x22), self.bus.read_byte_data(0x6b, 0x23))
    gy = self.byteToNumber(self.bus.read_byte_data(0x6b, 0x24), self.bus.read_byte_data(0x6b, 0x25))
    gz = self.byteToNumber(self.bus.read_byte_data(0x6b, 0x26), self.bus.read_byte_data(0x6b, 0x27))
    return (gx, gy, gz)

  
  def safe_smbus(self, action, tries, on_error):
    for _ in range(tries):
      try:
        return action()
      except KeyboardInterrupt:
        return
      except:
        time.sleep(0.0002)  # Sleep for 0.2 millis
        continue
    on_error()
    return None


  #### HIGH-LEVEL INTERFACE FUNCTIONS ####
  def write_motors(self, left, right, on_error = lambda : print("Error writing to morots")):
    func = lambda : self.write_pack(0, 'hh', left, right)           # Position 0, Length 4
    return self.safe_smbus(func, self.retry_max, on_error)
  
  def write_led_ring(self, red, green, blue, on_error = lambda : print("Error writing to led ring")):
    func = lambda : self.write_pack(4, 'BBB', red, green, blue)     # Position 4, Length 3
    return self.safe_smbus(func, self.retry_max, on_error)

  def write_alive(self, on_error = lambda : print("Error writing heartbeat")):
    self.alive = (self.alive + 1) % 255
    func = lambda : self.write_pack(7, 'B', self.alive)             # Position 7, Length 1
    return self.safe_smbus(func, self.retry_max, on_error)

  def read_encoders(self, on_error = lambda : print("Error reading from encoders")):
    func = lambda : self.read_unpack(8, 16, 'qq')                   # Position 8, Length 16
    return self.safe_smbus(func, self.retry_max, on_error)


  def read_gyroscope(self, on_error = lambda : print("Error reading from gyroscope")):
    
    # read back the gyro values
    func = lambda : self.read_gryo_internal()
    result = self.safe_smbus(func, self.retry_max, on_error)
    
    if result is None:
      return None
    
    gx, gy, gz = result
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
      number = number - 65536 #For two's complement
    return number