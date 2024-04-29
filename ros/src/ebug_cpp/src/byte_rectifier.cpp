#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"

#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

class ByteRectifier : public rclcpp::Node
{
    private:
        rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr m_CompressedImagePublisher;
        rclcpp::Subscriber<sensor_msgs::msg::Image>::SharedPtr m_ImageSubscriber;

    public:
        ByteRectifier() : Node("ByteRectifier")
        {
            m_CompressedImagePublisher = this->create_publisher<sensor_msgs::msg::CompressedImage>("image_compressed", 10);

            m_ImageSubscriber = this->create_subscription<sensor_msgs::msg::Image>("image_raw", 10, std::bind(&ByteRectifier::camera_callback, this, _1));
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


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ByteRectifier>());
  rclcpp::shutdown();
  return 0;
}