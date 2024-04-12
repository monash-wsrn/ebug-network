#include <rclcpp/rclcpp.hpp>
#include <grid_map_ros/grid_map_ros.hpp>
#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>
#include "nav_msgs/msg/odometry.hpp"

using std::placeholders::_1;

// Define the parameters of the Gaussian function
double sigma_x = 0.1; // Standard deviation in x direction
double sigma_y = 0.1; // Standard deviation in y direction
double A = 0.0000001;       // Amplitude


class OdomSubscriber : public rclcpp::Node
{
    public:
    OdomSubscriber()
    : Node("odom_subscriber")
    {
        subscription_ = this->create_subscription<nav_msgs::msg::Odometry>("odom", 10, std::bind(&OdomSubscriber::topic_callback, this, _1));

        publisher_ = this->create_publisher<grid_map_msgs::msg::GridMap>("grid_map", rclcpp::QoS(1).transient_local());

        clock_ = new rclcpp::Clock();
    }
    
    private:
    void topic_callback(const nav_msgs::msg::Odometry::SharedPtr msg) const
    {
        //RCLCPP_INFO(this->get_logger(), "Position-> x: [%f], y: [%f], z: [%f]", msg->pose.pose.position.x,msg->pose.pose.position.y, msg->pose.pose.position.z);

        rclcpp::Time time = this->now();
        grid_map::GridMap map({"elevation"});
        map.setFrameId("map");
        map.setGeometry(grid_map::Length(3.0, 3.0), 0.03);

        // Update map whenever a position changes
        for (grid_map::GridMapIterator it(map); !it.isPastEnd(); ++it) {
                grid_map::Position position;
                map.getPosition(*it, position);

                double distx = msg->pose.pose.position.x - position.x();  // X-axis distance (absolute)
                double disty = msg->pose.pose.position.y - position.y();  // Y-axis distance (absolute)
                
                // RCLCPP_INFO(this->get_logger(), "Robot Position-> x_r: [%f], y_r: [%f], z_r: [%f]", msg->pose.pose.position.x,msg->pose.pose.position.y, msg->pose.pose.position.z);
                // RCLCPP_INFO(this->get_logger(), "Map Position-> x_m: [%f], y_m: [%f]", position.x(),position.y());
                // RCLCPP_INFO(this->get_logger(), "Map Position-> x_d: [%f], y_d: [%f]", distx,disty);
                // double dist = std::sqrt(distx*distx + disty*disty) + 0.0000001;     // Vector distance (absolute)
                // double factor = std::pow(1.0 - dist, 6); // Ease out
                // map.at("elevation", *it) = 0.00001 * std::max(0.0, factor);
                
                map.at(
                "elevation",
                *it) = A * std::exp(-(distx * distx) / (2 * sigma_x * sigma_x) - 
                                    (disty * disty) / (2 * sigma_y * sigma_y));
            }

        
        map.setTimestamp(time.nanoseconds());
        std::unique_ptr<grid_map_msgs::msg::GridMap> message;
        message = grid_map::GridMapRosConverter::toMessage(map);
        publisher_->publish(std::move(message));

        //RCLCPP_INFO_THROTTLE(this->get_logger(), *clock_, 1000, "Grid map published.");
    }

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_;
    
    rclcpp::Publisher<grid_map_msgs::msg::GridMap>::SharedPtr publisher_;

    rclcpp::Clock* clock_;
};


int main(int argc, char ** argv)
{
    //Initialize node and publisher.
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OdomSubscriber>());
    return 0;   
}   
