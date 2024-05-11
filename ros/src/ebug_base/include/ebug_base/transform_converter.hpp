#ifndef EBUG_TRANSFORM_CONVERTER_HPP_
#define EBUG_TRANSFORM_CONVERTER_HPP_

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

#include <geometry_msgs/msg/transform.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>

#include <tf2/transform_datatypes.h>

#include <tf2/LinearMath/Scalar.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>

#include <tf2_msgs/msg/tf_message.hpp>


namespace ebug
{
    class TransformConverter : public rclcpp::Node
    {
        private:
            rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr m_Publisher;
            rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr m_Subscription;
            
            std::vector<tf2::Transform> m_Cameras;
            std::array<double, 36> m_Covariance;
        public:
            explicit TransformConverter(const rclcpp::NodeOptions& options);

        private:
            void transform_callback(const tf2_msgs::msg::TFMessage::ConstSharedPtr& detections) const;
    };
}
#endif  // EBUG_TRANSFORM_CONVERTER_HPP_