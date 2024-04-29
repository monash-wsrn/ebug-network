#ifndef EBUG_CAMERA_CONTROLLER_HPP_
#define EBUG_CAMERA_CONTROLLER_HPP_

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
#include <sensor_msgs/msg/camera_info.hpp>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

namespace ebug
{
    class CameraController : public rclcpp::Node
    {
        typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image, sensor_msgs::msg::CameraInfo> approximate_policy;
        
        private:
            rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr m_CameraInfoPublisher;
            rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr m_ImagePublisher;

            std::vector<std::string> m_Cameras;
            std::vector<std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Image>>> m_Images;
            std::vector<std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::CameraInfo>>> m_CameraInfos;
            std::vector<std::shared_ptr<message_filters::Synchronizer<approximate_policy>>> m_Synchronizers;

        public:
            EBUG_EXPORT
            explicit CameraController(const rclcpp::NodeOptions& options);

        private:
            void camera_callback(const sensor_msgs::msg::Image::ConstSharedPtr& image, const sensor_msgs::msg::CameraInfo::ConstSharedPtr& cinfo) const;
    };
}
#endif  // EBUG_CAMERA_CONTROLLER_HPP_