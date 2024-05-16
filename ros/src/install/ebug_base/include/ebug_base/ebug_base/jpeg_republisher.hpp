#ifndef EBUG_JPEG_REPUBLISHER_HPP_
#define EBUG_JPEG_REPUBLISHER_HPP_

#ifdef __GNUC__
    #define EBUG_EXPORT __attribute__ ((dllexport))
#else
    #define EBUG_EXPORT __declspec(dllexport)
#endif

#include <memory>
#include <string>
#include <utility>

#include "rclcpp/rclcpp.hpp"

#include "pluginlib/class_loader.hpp"

#include "image_transport/image_transport.hpp"
#include "image_transport/publisher_plugin.hpp"
#include "image_transport/subscriber.hpp"

namespace ebug
{
    typedef image_transport::PublisherPlugin Plugin;

    class JpegRepublisher : public rclcpp::Node
    {
        private:
            std::shared_ptr<Plugin> m_Publisher;
            image_transport::Subscriber m_Subscriber;
            
            std::shared_ptr<pluginlib::ClassLoader<Plugin>> m_Loader;

        public:
            explicit JpegRepublisher(const rclcpp::NodeOptions& options);
    };
}
#endif  // EBUG_JPEG_REPUBLISHER_HPP_