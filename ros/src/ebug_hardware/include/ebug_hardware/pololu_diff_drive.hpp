#ifndef EBUG_POLOLU_DIFF_DRIVE_HPP_
#define EBUG_POLOLU_DIFF_DRIVE_HPP_

#ifdef __GNUC__
    #define EBUG_EXPORT __attribute__ ((dllexport))
#else
    #define EBUG_EXPORT __declspec(dllexport)
#endif

#include <chrono>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/time.hpp"

#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"

#include "romi_i2c_bridge.hpp"

#include "pololu_wheel.hpp"
#include "pololu_lights.hpp"

namespace ebug
{
    struct PololuConfig {
        std::string left_wheel_name = "left_wheel";
        std::string right_wheel_name = "right_wheel";
        std::string device = "/dev/i2c-1";
        
        float loop_rate = 30;
        double encoder_cpr = 1440;
    };

    using hardware_interface::return_type;
    using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

    class PololuDiffDrive : public hardware_interface::SystemInterface
    {
    private:
        PololuConfig m_Config;

        RomiI2CBridge m_Bridge;

        PololuWheel m_WheelLeft;
        PololuWheel m_WheelRight;

        PololuLights m_Lights;

        rclcpp::Logger m_Logger;

    public:
        RCLCPP_SHARED_PTR_DEFINITIONS(PololuDiffDrive);

        PololuDiffDrive();

        EBUG_EXPORT
        CallbackReturn on_init(const hardware_interface::HardwareInfo& info) override;
        
        
        EBUG_EXPORT 
        std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
        
        EBUG_EXPORT
        std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;


        EBUG_EXPORT
        CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
        
        EBUG_EXPORT
        CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;
        
        EBUG_EXPORT
        return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;
        
        EBUG_EXPORT
        return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;
    };
}
#endif  // EBUG_POLOLU_DIFF_DRIVE_HPP_