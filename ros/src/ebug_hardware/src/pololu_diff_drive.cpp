#include "ebug_hardware/pololu_diff_drive.hpp"

using namespace std::chrono_literals;

namespace ebug
{
    PololuDiffDrive::PololuDiffDrive() : m_Logger(rclcpp::get_logger("PololuDiffDrive"))
    {

    }

    CallbackReturn PololuDiffDrive::on_init(const hardware_interface::HardwareInfo& info)
    {
        if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS)
            return CallbackReturn::ERROR;

        m_Config.left_wheel_name = info_.hardware_parameters["left_wheel_name"];
        m_Config.right_wheel_name = info_.hardware_parameters["right_wheel_name"];

        m_Config.device = info_.hardware_parameters["device"];
        
        m_Config.loop_rate = std::stof(info_.hardware_parameters["loop_rate"]);
        m_Config.encoder_cpr = std::stod(info_.hardware_parameters["encoder_cpr"]);

        m_WheelLeft.configure(m_Config.left_wheel_name, m_Config.encoder_cpr);
        m_WheelRight.configure(m_Config.right_wheel_name, m_Config.encoder_cpr);

        m_Bridge.configure(m_Config.device);        
        return CallbackReturn::SUCCESS;
    }

    std::vector<hardware_interface::StateInterface> PololuDiffDrive::export_state_interfaces()
    {
        // We need to set up a position and a velocity interface for each wheel
        std::vector<hardware_interface::StateInterface> ifs;

        ifs.emplace_back(hardware_interface::StateInterface(m_WheelLeft.name, hardware_interface::HW_IF_VELOCITY, &m_WheelLeft.vel));
        ifs.emplace_back(hardware_interface::StateInterface(m_WheelLeft.name, hardware_interface::HW_IF_POSITION, &m_WheelLeft.pos));

        ifs.emplace_back(hardware_interface::StateInterface(m_WheelRight.name, hardware_interface::HW_IF_VELOCITY, &m_WheelRight.vel));
        ifs.emplace_back(hardware_interface::StateInterface(m_WheelRight.name, hardware_interface::HW_IF_POSITION, &m_WheelRight.pos));

        return ifs;
    }

    std::vector<hardware_interface::CommandInterface> PololuDiffDrive::export_command_interfaces() 
    {
        // We need to set up a velocity command interface for each wheel
        std::vector<hardware_interface::CommandInterface> ifs;

        ifs.emplace_back(hardware_interface::CommandInterface(m_WheelLeft.name, hardware_interface::HW_IF_VELOCITY, &m_WheelLeft.cmd));
        ifs.emplace_back(hardware_interface::CommandInterface(m_WheelRight.name, hardware_interface::HW_IF_VELOCITY, &m_WheelRight.cmd));

        // ifs.emplace_back(hardware_interface::CommandInterface(m_Lights.name, hardware_interface::HW_IF_POSITION, &m_Lights.RGBA));

        return ifs;
    }

    
    
    CallbackReturn PololuDiffDrive::on_activate(const rclcpp_lifecycle::State & previous_state)
    {
        return CallbackReturn::SUCCESS;
    }
        
    CallbackReturn PololuDiffDrive::on_deactivate(const rclcpp_lifecycle::State & previous_state)
    {
        m_Bridge.writeMotors(0, 0);
        m_Bridge.writeLights(0, 0, 0);

        return CallbackReturn::SUCCESS;
    }

    return_type PololuDiffDrive::read(const rclcpp::Time & time, const rclcpp::Duration & period) 
    {
        if ( !m_Bridge.available() )
            return return_type::ERROR;

        int16_t lenc, renc;
        m_Bridge.readEncoders(lenc, renc);

        m_WheelLeft.recalculate(lenc, period.seconds());
        m_WheelRight.recalculate(renc, period.seconds());

        return return_type::OK;
    }
    
    return_type PololuDiffDrive::write(const rclcpp::Time & time, const rclcpp::Duration & period) 
    {
        if ( !m_Bridge.available() )
            return return_type::ERROR;
        
        int16_t left = m_WheelLeft.cmd / m_WheelLeft.rads_per_count / m_Config.loop_rate;  
        int16_t right = m_WheelRight.cmd / m_WheelRight.rads_per_count / m_Config.loop_rate;
        m_Bridge.writeMotors(left, right);

        uint8_t r = (uint8_t) (m_Lights.RGBA >> 24);
        uint8_t g = (uint8_t) (m_Lights.RGBA >> 16);
        uint8_t b = (uint8_t) (m_Lights.RGBA >>  8);
        m_Bridge.writeLights(r, g, b);

        return return_type::OK;
    }
}


#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(ebug::PololuDiffDrive, hardware_interface::SystemInterface)