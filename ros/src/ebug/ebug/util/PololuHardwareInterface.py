# Copyright Pololu Corporation.  For more information, see https://www.pololu.com/

# gyroscope code reference
# 1. https://github.com/DarkSparkAg/MinIMU-9-v5/blob/master/MinIMU_v5_pi.py
# 2. http://rpi-data.blogspot.com/2016/09/programming-i2c-bus-in-python.html

import smbus  # Library to interface with I2C devices
import struct  # Library to handle C-like data structures in Python
import time  # Library for time-related functions
import math  # Library for mathematical functions

class PololuHardwareInterface:
    def __init__(self, retry_max, logger=None):
        self.retry_max = retry_max  # Maximum number of retries for I2C communication
        self.bus = smbus.SMBus(1)  # Initialize I2C bus (1 refers to /dev/i2c-1)
        self.logger = logger  # Accept logger from the ROS node

        # Initialize the gyroscope
        self.bus.write_byte_data(0x6b, 0x16, 0x38)  # Set gyroscope to active mode
        self.bus.write_byte_data(0x6b, 0x11, 0x60)  # Set gyroscope sensitivity to 245dps
        self.G_GAIN = 0.00875  # Sensitivity gain

        self.time_prev = time.time()  # Record the initial time

        self.alive = 0  # Heartbeat counter
        self.write_alive()  # Send the first heartbeat signal
    
    def log_info(self, message):
        if self.logger:
            self.logger.info(message)
        else:
            print(message)

    def read_unpack(self, address, size, format):
        """
        Read and unpack data from the given I2C address.

        :param address: I2C address to read from
        :param size: Number of bytes to read
        :param format: The format string used for struct.unpack()
        :return: Unpacked data
        """
        self.bus.write_byte(20, address)  # Write the address to the I2C bus
        time.sleep(0.0002)  # Delay for 0.2 milliseconds

        # Read the specified number of bytes and unpack them according to the format
        byte_list = [self.bus.read_byte(20) for _ in range(size)]
        return struct.unpack(format, bytes(byte_list))

    def write_pack(self, address, format, *data):
        """
        Pack and write data to the given I2C address.

        :param address: I2C address to write to
        :param format: The format string used for struct.pack()
        :param data: Data to pack and write
        """
        data_array = list(struct.pack(format, *data))  # Pack the data into a byte array
        self.bus.write_i2c_block_data(20, address, data_array)  # Write the byte array to the I2C bus
        time.sleep(0.0002)  # Delay for 0.2 milliseconds

    def read_gryo_internal(self):
        """
        Read internal gyroscope data (raw values).

        :return: Tuple containing raw gyroscope data for x, y, and z axes
        """
        gx = self.byteToNumber(self.bus.read_byte_data(0x6b, 0x22), self.bus.read_byte_data(0x6b, 0x23))
        gy = self.byteToNumber(self.bus.read_byte_data(0x6b, 0x24), self.bus.read_byte_data(0x6b, 0x25))
        gz = self.byteToNumber(self.bus.read_byte_data(0x6b, 0x26), self.bus.read_byte_data(0x6b, 0x27))
        return (gx, gy, gz)

    def safe_smbus(self, action, tries, on_error):
        """
        Perform an I2C action safely with retries.

        :param action: The I2C action to perform
        :param tries: Maximum number of retry attempts
        :param on_error: Error handling function
        :return: Result of the I2C action or None if it fails
        """
        for _ in range(tries):
            try:
                return action()
            except KeyboardInterrupt:
                return  # Allow user to interrupt with Ctrl+C
            except:
                time.sleep(0.0002)  # Delay before retrying
                continue
        on_error()  # Call the error handler if all retries fail
        return None

    #### HIGH-LEVEL INTERFACE FUNCTIONS ####

    def read_odometry(self, on_error=None):
        """
        Read odometry values from the Arduino.
        :param on_error: Error handling function
        :return: Tuple containing all odometry values (linear_vel, angular_vel, rled, gled, bled, alive, x, y, theta)
        """
        if on_error is None:
            on_error = lambda: self.log_info("Error reading odometry")

        def func():
            return self.read_unpack(12, 20, 'fffff')  # Read from address 12, 20 bytes

        result = self.safe_smbus(func, self.retry_max, on_error)
        if result is None:
            self.log_info("Failed to read odometry after maximum retries")
        return result

    def write_motors(self, left, right, on_error=lambda: print("Error writing to motors")):
        """
        Write motor PWM values to the I2C bus.

        :param left: PWM value for the left motor
        :param right: PWM value for the right motor
        :param on_error: Error handling function
        """
        func = lambda: self.write_pack(0, 'hh', left, right)  # Pack the PWM values as two short integers
        return self.safe_smbus(func, self.retry_max, on_error)

    def write_led_ring(self, red, green, blue, on_error=lambda: print("Error writing to LED ring")):
        """
        Write RGB values to the LED ring via the I2C bus.

        :param red: Red intensity (0-255)
        :param green: Green intensity (0-255)
        :param blue: Blue intensity (0-255)
        :param on_error: Error handling function
        """
        func = lambda: self.write_pack(4, 'BBB', red, green, blue)  # Pack the RGB values as three bytes
        return self.safe_smbus(func, self.retry_max, on_error)

    def write_alive(self, on_error=lambda: print("Error writing heartbeat")):
        """
        Send a heartbeat signal to the Arduino.

        :param on_error: Error handling function
        """
        self.alive = (self.alive + 1) % 255  # Increment the heartbeat counter, looping back to 0 after 255
        func = lambda: self.write_pack(7, 'B', self.alive)  # Pack the heartbeat as a single byte
        return self.safe_smbus(func, self.retry_max, on_error)

    def reset_odometry(self, on_error=lambda: print("Error resetting odometry")):
        """Reset odometry values on Arduino"""
        def func():
            # Write to a specific reset address/register
            self.write_pack(9, 'B', 1)  # Using address 10 for reset command
        return self.safe_smbus(func, self.retry_max, on_error)

    def write_velocity(self, linear_velocity, angular_velocity, on_error=lambda: print("Error writing velocity to motors")):
        """
        Write desired velocities for the motors to the I2C bus.

        :param linear_velocity: Desired linear velocity
        :param angular_velocity: Desired angular velocity 
        :param on_error: Error handling function
        """
        self.log_info(f"Sending linear velocity: {linear_velocity}, angular velocity: {angular_velocity}")
        func = lambda: self.write_pack(0, 'ff', linear_velocity, angular_velocity)  # Pack the velocities as two floats
        return self.safe_smbus(func, self.retry_max, on_error)

    def read_encoders(self, on_error=lambda: print("Error reading from encoders")):
        """
        Read encoder values from the Arduino.

        :param on_error: Error handling function
        :return: Tuple containing the encoder values for the left and right motors
        """
        func = lambda: self.read_unpack(8, 16, 'qq')  # Unpack two long long integers (64-bit) from the I2C bus
        return self.safe_smbus(func, self.retry_max, on_error)

    def read_gyroscope(self, on_error=lambda: print("Error reading from gyroscope")):
        """
        Read processed gyroscope data (in radians per second).

        :param on_error: Error handling function
        :return: Tuple containing angular velocities for x, y, and z axes in radians per second
        """
        func = lambda: self.read_gryo_internal()
        result = self.safe_smbus(func, self.retry_max, on_error)

        if result is None:
            return None

        gx, gy, gz = result
        # Apply the sensitivity gain and correct for constant offset
        wx = math.radians(gx * self.G_GAIN - 4)
        wy = math.radians(gy * self.G_GAIN + 8)
        wz = math.radians(gz * self.G_GAIN + 5)

        return (wx, wy, wz)  # Return angular velocities in radians per second

    def byteToNumber(self, val_Low, val_Hi):
        """
        Convert two 8-bit values (low and high bytes) to a 16-bit signed integer.

        :param val_Low: Low byte
        :param val_Hi: High byte
        :return: 16-bit signed integer
        """
        number = 256 * val_Hi + val_Low  # Combine the two bytes into a 16-bit number
        if number >= 32768:  # If the number is greater than or equal to 32768, it's negative in two's complement
            number = number - 65536  # Convert from two's complement to a negative integer
        return number
