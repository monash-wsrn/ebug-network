#include "ebug_cpp/jpeg_republisher.hpp"

using namespace std::chrono_literals;

namespace ebug
{
    JpegRepublisher::JpegRepublisher(const rclcpp::NodeOptions& options) : Node("JpegRepublisher", options)
    {
        std::string in_topic = rclcpp::expand_topic_or_service_name("in", this->get_name(), this->get_namespace());
        std::string out_topic = rclcpp::expand_topic_or_service_name("out", this->get_name(), this->get_namespace());

        std::string in_transport = this->declare_parameter<std::string>("in_transport", "compressed");
        std::string out_transport = this->declare_parameter<std::string>("out_transport", "raw");

        // Load transport plugin
        pluginlib::ClassLoader<Plugin> loader("image_transport", "image_transport::PublisherPlugin");
        std::string lookup_name = Plugin::getLookupName(out_transport);

        auto instance = loader.createUniqueInstance(lookup_name);
        m_Publisher = std::move(instance);
        m_Publisher->advertise(this, out_topic);

        // Use PublisherPlugin::publish as the subscriber callback
        typedef void (Plugin::* PublishMemFn)(const sensor_msgs::msg::Image::ConstSharedPtr &) const;
        PublishMemFn pub_mem_fn = &Plugin::publishPtr;
        m_Subscriber = image_transport::create_subscription(this, in_topic, std::bind(pub_mem_fn, m_Publisher.get(), std::placeholders::_1), in_transport);
    }
}


#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(ebug::JpegRepublisher)