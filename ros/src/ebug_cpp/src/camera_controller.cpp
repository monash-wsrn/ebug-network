#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"

#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

class CameraController : public rclcpp::Node
{
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image, sensor_msgs::msg::CameraInfo> approximate_policy;

    private:
        rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr m_CameraInfoPublisher;
        rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr m_CompressedImagePublisher;

        std::vector<std::string> m_Cameras;
        std::vector<message_filters::Subscriber<sensor_msgs::msg::Image>> m_Images;
        std::vector<message_filters::Subscriber<sensor_msgs::msg::CameraInfo>> m_Cinfos;
        std::vector<std::shared_ptr<message_filters::Synchronizer<approximate_policy>>> m_Synchronizers;

    public:
        CameraController() : Node("CameraController")
        {
            m_Cameras = this->declare_parameter<std::vector<std::string>>("cameras", std::vector<std::string>( { "cam_0" } ));

            m_CompressedImagePublisher = this->create_publisher<sensor_msgs::msg::CompressedImage>("image_raw/compressed", 10);
            m_CameraInfoPublisher = this->create_publisher<sensor_msgs::msg::CameraInfo>("camera_info", 10);

            for (auto &cam_id : m_Cameras) 
            {
                message_filters::Subscriber<sensor_msgs::msg::Image> image(this, cam_id + "/image_raw");
                m_Images.push_back(image);
                image.subscribe();

                message_filters::Subscriber<sensor_msgs::msg::CameraInfo> cinfo(this, cam_id + "/camera_info");
                m_Cinfos.push_back(imacinfoge);
                cinfo.subscribe();

                auto sync = std::make_shared<message_filters::Synchronizer<approximate_policy>>(approximate_policy(10), image, cinfo);
                m_Synchronizers.push_back(sync);
                
                sync->setMaxIntervalDuration(rclcpp::Duration(0,100000000)); // 0.1 Second interval
                sync->registerCallback(std::bind(&CameraController::camera_callback, this, std::placeholders::_1, std::placeholders::_2));

                RCLCPP_INFO(this->get_logger(), "Connected USB Camera (ID: %s)", cam_id.c_str());
            }
        }

    private:
        void camera_callback(const sensor_msgs::msg::Image::ConstSharedPtr& image, const sensor_msgs::msg::CameraInfo::ConstSharedPtr& cinfo) const
        {
            RCLCPP_INFO(this->get_logger(), "Received Image & CameraInfo");

            sensor_msgs::msg::CompressedImage cimage;

            cimage.header = image->header;
            cimage.data = image->data;

            cimage.format = "jpeg";
            
            m_CompressedImagePublisher->publish(cimage);
            m_CameraInfoPublisher->publish(*cinfo);
        }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CameraController>());
  rclcpp::shutdown();
  return 0;
}