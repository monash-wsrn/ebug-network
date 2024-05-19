#include "ebug_hardware/romi_i2c_bridge.hpp"

using namespace std::chrono_literals;

namespace ebug
{
    RomiI2CBridge::RomiI2CBridge()
    {
        m_Available = false;
    }

    RomiI2CBridge::~RomiI2CBridge()
    {
        if (m_File >= 0)
            close(m_File);
    }

    void RomiI2CBridge::configure(const std::string& device)
    {
        m_Device = device;
        m_File = open(m_Device.c_str(), O_RDWR);

        if (m_File < 0)
            return;
        
        if (ioctl(m_File, I2C_SLAVE, I2C_ADDRESS) < 0)
            return;

        m_Available = true;
    }

    void RomiI2CBridge::writeMotors(int16_t left, int16_t right)
    {
        i2c_smbus_write_word_data(m_File, 0x00, (uint16_t) left);
        i2c_smbus_write_word_data(m_File, 0x02, (uint16_t) right);
    }
    
    void RomiI2CBridge::writeLights(uint8_t red, uint8_t green, uint8_t blue)
    {
        uint8_t RGB[3] = { red, green, blue };
        i2c_smbus_write_block_data(m_File, 0x04, 0x03, RGB);
    }

    void RomiI2CBridge::readEncoders(int16_t& left, int16_t& right)
    {
        __s32 l = i2c_smbus_read_word_data(m_File, 0x07);
        __s32 r = i2c_smbus_read_word_data(m_File, 0x09);
        
        left = (int16_t) l;
        right = (int16_t) r;
    }

    bool RomiI2CBridge::available() 
    {
        return m_Available;
    }
}