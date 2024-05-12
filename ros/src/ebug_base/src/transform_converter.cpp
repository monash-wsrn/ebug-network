#include "ebug_base/transform_converter.hpp"

using namespace std::chrono_literals;

namespace ebug
{
    TransformConverter::TransformConverter(const rclcpp::NodeOptions& options) : Node("TransformConverter", options)
    {
        m_Publisher = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("pose", 
            rclcpp::QoS(1).best_effort().durability_volatile());

        m_Subscription = this->create_subscription<tf2_msgs::msg::TFMessage>("tf_detections", 
            rclcpp::QoS(1).reliable().durability_volatile(), 
            std::bind(&TransformConverter::transform_callback, this, std::placeholders::_1));

        m_Covariance = {
            0.1, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.1, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.1, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.1, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.1, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.1
        };

        tf2::Vector3 offset(0.0, 0.0, -0.025);


        tf2::Quaternion rot_0(0.0, 0.0, 0.0, 0.0);
        tf2::Transform cam_0(rot_0, offset);
        m_Cameras.push_back(cam_0);

        tf2::Quaternion rot_1(0.0, 0.0, 0.7071068, 0.7071068);
        tf2::Transform cam_1(rot_1, offset);
        m_Cameras.push_back(cam_1);

        tf2::Quaternion rot_2(0.0, 0.0, 1.0, 0.0);
        tf2::Transform cam_2(rot_2, offset);
        m_Cameras.push_back(cam_2);

        tf2::Quaternion rot_3(0.0, 0.0, 0.7071068, -0.7071068);
        tf2::Transform cam_3(rot_3, offset);
        m_Cameras.push_back(cam_3);
    }

    void TransformConverter::transform_callback(const tf2_msgs::msg::TFMessage::ConstSharedPtr& detections) const
    {
        auto tfs = detections->transforms;
        RCLCPP_INFO(this->get_logger(), "Callback A ->  %d", tfs.size());


        for(const geometry_msgs::msg::TransformStamped& ts : tfs) 
        {
            RCLCPP_INFO(this->get_logger(), "Callback B");            

            const int cam_id = (int)(ts.header.frame_id.back() - '0');
            
            tf2::Vector3 pos(ts.transform.translation.x, ts.transform.translation.y, ts.transform.translation.z);
            tf2::Quaternion rot(ts.transform.rotation.x, ts.transform.rotation.y, ts.transform.rotation.z, ts.transform.rotation.w);
            tf2::Transform tag_cam(rot, pos);

            tf2::Transform robot_tag = m_Cameras[cam_id] * tag_cam.inverse();
            
            RCLCPP_INFO(this->get_logger(), "Callback C");
            
            geometry_msgs::msg::PoseWithCovarianceStamped msg;
            msg.header.stamp = ts.header.stamp;
            msg.header.frame_id = ts.child_frame_id;
            
            msg.pose.pose.position.x = (double) robot_tag.getOrigin().getX();
            msg.pose.pose.position.y = (double) robot_tag.getOrigin().getY();
            msg.pose.pose.position.z = (double) robot_tag.getOrigin().getZ();

            msg.pose.pose.orientation.x = (double) robot_tag.getRotation().getAxis().getX();
            msg.pose.pose.orientation.y = (double) robot_tag.getRotation().getAxis().getY();
            msg.pose.pose.orientation.z = (double) robot_tag.getRotation().getAxis().getZ();
            msg.pose.pose.orientation.w = (double) robot_tag.getRotation().getW();

            msg.pose.covariance = m_Covariance;
            
            RCLCPP_INFO(this->get_logger(), "Callback D");

            m_Publisher->publish(std::move(msg));
            
            RCLCPP_INFO(this->get_logger(), "Callback E");
            
            // RCLCPP_INFO(this->get_logger(), "%.4f %.4f %.4f", robot_tag.getOrigin().getX(), robot_tag.getOrigin().getY(), robot_tag.getOrigin().getZ());
        }
    }
}


#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(ebug::TransformConverter)