#ifndef EBUG_GRIDMAP_CONTROLLER_HPP_
#define EBUG_GRIDMAP_CONTROLLER_HPP_

#ifdef __GNUC__
    #define EBUG_EXPORT __attribute__ ((dllexport))
#else
    #define EBUG_EXPORT __declspec(dllexport)
#endif

#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <grid_map_ros/grid_map_ros.hpp>

#include "ebug_base/msg/robot_pose.hpp" 

namespace ebug
{
    class GridmapController : public rclcpp::Node
    {
        private:
            std::unordered_map<std::string, std::pair<geometry_msgs::msg::Pose, std::pair<double, double>>> robot_poses_;
            rclcpp::Subscription<ebug_base::msg::RobotPose>::SharedPtr subscription_;
            rclcpp::Publisher<grid_map_msgs::msg::GridMap>::SharedPtr publisher_;

        public:
            explicit GridmapController(const rclcpp::NodeOptions& options);

        private:
            void topic_callback(const ebug_base::msg::RobotPose::SharedPtr msg);
            void update_grid_map(); 
    };
}
#endif  // EBUG_GRIDMAP_CONTROLLER_HPP_