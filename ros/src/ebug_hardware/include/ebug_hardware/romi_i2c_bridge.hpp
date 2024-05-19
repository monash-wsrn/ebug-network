#ifndef EBUG_ROMI_I2C_BRIDGE_HPP_
#define EBUG_ROMI_I2C_BRIDGE_HPP_

#ifdef __GNUC__
    #define EBUG_EXPORT __attribute__ ((dllexport))
#else
    #define EBUG_EXPORT __declspec(dllexport)
#endif

#include <chrono>
#include <string>
#include <vector>

#include <stdint.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/stat.h>
#include <unistd.h>

extern "C" {
    #include <sys/ioctl.h>
    #include <linux/i2c-dev.h>
    #include <linux/i2c.h>

    #include <i2c/smbus.h>
}

#define I2C_ADDRESS 0x14

namespace ebug
{
    class RomiI2CBridge
    {
    private:
        std::string m_Device;
        int m_File;
        bool m_Available;

    public:
        RomiI2CBridge();
        ~RomiI2CBridge();

        void configure(const std::string& device = "/devi/i2c-1");

        void writeMotors(int16_t left, int16_t right);
        void writeLights(uint8_t red, uint8_t green, uint8_t blue);

        void readEncoders(int16_t& left, int16_t& right);

        bool available();
    };
}
#endif  // EBUG_ROMI_I2C_BRIDGE_HPP_