#include "ebug_cpp/camera_controller.hpp"

using namespace std::chrono_literals;

namespace ebug
{

    CameraController::CameraController(const rclcpp::NodeOptions& options) : Node("CameraController", options)
    {
        m_Cameras = this->declare_parameter<std::vector<std::string>>("cameras", std::vector<std::string>( { "cam_0" } ));

        m_ImagePublisher = this->create_publisher<sensor_msgs::msg::Image>("image_rect", 10);
        m_CameraInfoPublisher = this->create_publisher<sensor_msgs::msg::CameraInfo>("camera_info", 10);

        for (auto &cam_id : m_Cameras) 
        {
            auto image = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image>>(this, cam_id + "/image_rect");
            image->subscribe();
            m_Images.push_back(image);

            auto cinfo = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::CameraInfo>>(this, cam_id + "/camera_info");
            m_CameraInfos.push_back(cinfo);
            cinfo->subscribe();

            auto sync = std::make_shared<message_filters::Synchronizer<approximate_policy>>(approximate_policy(10), *image, *cinfo);
            m_Synchronizers.push_back(sync);
            
            sync->setMaxIntervalDuration(rclcpp::Duration(0,100000000)); // 0.1 Second interval
            sync->registerCallback(std::bind(&CameraController::camera_callback, this, std::placeholders::_1, std::placeholders::_2));

            RCLCPP_INFO(this->get_logger(), "Connected USB Camera (ID: %s)", cam_id.c_str());
        }
    }

    void CameraController::camera_callback(const sensor_msgs::msg::Image::ConstSharedPtr& image, const sensor_msgs::msg::CameraInfo::ConstSharedPtr& cinfo) const
    {            
        m_ImagePublisher->publish(*image);
        m_CameraInfoPublisher->publish(*cinfo);
    }
}


#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(ebug::CameraController)