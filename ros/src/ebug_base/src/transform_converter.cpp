#include "ebug_base/transform_converter.hpp"

using namespace std::chrono_literals;

namespace ebug
{
    TransformConverter::TransformConverter(const rclcpp::NodeOptions& options) : Node("TransformConverter", options)
    {
        m_Publisher = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("pose", 10);

        m_Subscription = this->create_subscription<tf2_msgs::msg::TFMessage>("tf_detections", 10, std::bind(&TransformConverter::transform_callback, this, std::placeholders::_1));

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
        // RCLCPP_INFO(this->get_logger(), "Callback reached");

        for(const geometry_msgs::msg::TransformStamped& ts : detections->transforms) 
        {            
            const int cam_id = (int)(ts.header.frame_id.back() - '0');
            
            // RCLCPP_INFO(this->get_logger(), "    Iterating stamped transforms from camera %d", cam_id);
            
            tf2::Vector3 pos(ts.transform.translation.x, ts.transform.translation.y, ts.transform.translation.z);
            tf2::Quaternion rot(ts.transform.rotation.x, ts.transform.rotation.y, ts.transform.rotation.z, ts.transform.rotation.w);
            tf2::Transform tag_cam(rot, pos);

            tf2::Transform cam_tag = tag_cam.inverse();
            tf2::Transform robot_cam = m_Cameras[cam_id];
            tf2::Transform robot_tag = robot_cam * cam_tag;

            
            // RCLCPP_INFO(this->get_logger(), "    Creating adjusted transform for tag %s %.6f %.6f", ts.child_frame_id.c_str(), tag_cam.getOrigin().getZ(), robot_tag.getOrigin().getZ());
            geometry_msgs::msg::PoseWithCovarianceStamped msg;
            msg.header.stamp = ts.header.stamp;
            msg.header.frame_id = ts.child_frame_id;
            
            msg.pose.pose.position.x = robot_tag.getOrigin().getX();
            msg.pose.pose.position.y = robot_tag.getOrigin().getY();
            msg.pose.pose.position.z = robot_tag.getOrigin().getZ();

            msg.pose.pose.orientation.x = robot_tag.getRotation().getAxis().getX();
            msg.pose.pose.orientation.y = robot_tag.getRotation().getAxis().getY();
            msg.pose.pose.orientation.z = robot_tag.getRotation().getAxis().getZ();
            msg.pose.pose.orientation.w = robot_tag.getRotation().getW();
            
            m_Publisher->publish(msg);
        }
    }
}


#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(ebug::TransformConverter)