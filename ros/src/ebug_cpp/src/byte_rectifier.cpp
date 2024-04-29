#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"

#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>

using namespace std::chrono_literals;

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(ebug::ByteRectifier)


namespace ebug
{
    class ByteRectifier : public rclcpp::Node
    {
        private:
            rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr m_CompressedImagePublisher;
            rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr m_ImageSubscription;

        public:
            ByteRectifier(const rclcpp::NodeOptions& options) : Node("ByteRectifier", options)
            {
                m_CompressedImagePublisher = this->create_publisher<sensor_msgs::msg::CompressedImage>("image_compressed", 10);

                m_ImageSubscription = this->create_subscription<sensor_msgs::msg::Image>("image_raw", 10, std::bind(&ByteRectifier::camera_callback, this, std::placeholders::_1));
            }

        private:
            void camera_callback(const sensor_msgs::msg::Image::ConstSharedPtr& image) const
            {
                sensor_msgs::msg::CompressedImage cimage;

                cimage.header = image->header;
                cimage.data = image->data;

                cimage.format = "jpeg";
                
                m_CompressedImagePublisher->publish(cimage);
            }
    };
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ebug::ByteRectifier>());
  rclcpp::shutdown();
  return 0;
}