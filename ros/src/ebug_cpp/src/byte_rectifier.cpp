#include "ebug_cpp/byte_rectifier.hpp"

using namespace std::chrono_literals;

namespace ebug
{
    ByteRectifier::ByteRectifier(const rclcpp::NodeOptions& options) : Node("ByteRectifier", options)
    {
        m_CompressedImagePublisher = this->create_publisher<sensor_msgs::msg::CompressedImage>("image_compressed", 10);

        m_ImageSubscription = this->create_subscription<sensor_msgs::msg::Image>("image_raw", 10, std::bind(&ByteRectifier::camera_callback, this, std::placeholders::_1));
    }

    void ByteRectifier::camera_callback(const sensor_msgs::msg::Image::ConstSharedPtr& image) const
    {
        sensor_msgs::msg::CompressedImage cimage;

        cimage.header = image->header;
        cimage.data = image->data;

        cimage.format = "jpeg";
        
        m_CompressedImagePublisher->publish(cimage);
    }
}


#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(ebug::ByteRectifier)