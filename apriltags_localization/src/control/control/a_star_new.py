"""
Attempts to move PID controller to ROmi board, not working yet
"""

# Copyright Pololu Corporation.  For more information, see https://www.pololu.com/
import smbus
import struct
import time

class AStar:
    def __init__(self):
        self.bus = smbus.SMBus(1)

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
        time.sleep(0.0001)
        byte_list = [self.bus.read_byte(20) for _ in range(size)]
        return struct.unpack(format, bytes(byte_list))

    def write_pack(self, address, format, *data):
        data_array = list(struct.pack(format, *data))
        self.bus.write_i2c_block_data(20, address, data_array)
        time.sleep(0.0001)

    def read_battery_millivolts(self):
        return self.read_unpack(10, 2, "H")

    def set_speed(self, wl_desired, wr_desired):

        self.write_pack(0, 'f', float(wl_desired))
        self.write_pack(4, 'f', float(wr_desired))

    def read_wr(self):

        return self.read_unpack(8, 4, 'f')

    def read_wl(self):

        return self.read_unpack(12, 4, 'f')

    def test_read8(self):
        self.read_unpack(0, 8, 'cccccccc')

    def test_write8(self):
        self.bus.write_i2c_block_data(20, 0, [0,0,0,0,0,0,0,0])
        time.sleep(0.0001)
