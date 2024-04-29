#ifndef EBUG_BYTE_RECTIFIER_HPP_
#define EBUG_BYTE_RECTIFIER_HPP_

#ifdef __GNUC__
    #define EBUG_EXPORT __attribute__ ((dllexport))
#else
    #define EBUG_EXPORT __declspec(dllexport)
#endif

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"

#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>

namespace ebug
{
    class ByteRectifier : public rclcpp::Node
    {
        private:
            rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr m_CompressedImagePublisher;
            rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr m_ImageSubscription;

        public:
            EBUG_EXPORT
            explicit ByteRectifier(const rclcpp::NodeOptions& options);

        private:
            void camera_callback(const sensor_msgs::msg::Image::ConstSharedPtr& image) const;
    };
}
#endif  // EBUG_BYTE_RECTIFIER_HPP_