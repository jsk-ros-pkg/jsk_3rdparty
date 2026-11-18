#include "zdepth_image_transport/ros_version.h"
#if ROS_VERSION_MAJOR == 1
#include <image_transport/simple_publisher_plugin.h>
#else
#include <image_transport/simple_publisher_plugin.hpp>
#endif
#include "zdepth/zdepth.hpp"
#if ROS_VERSION_MAJOR == 1
#include "zdepth_image_transport/ZDepthImage.h"
#else
#include "zdepth_image_transport/msg/z_depth_image.hpp"
#endif


namespace zdepth_image_transport
{
#if ROS_VERSION_MAJOR == 1
  class ZDepthPublisher : public image_transport::SimplePublisherPlugin<zdepth_image_transport::ZDepthImage>
#else
  class ZDepthPublisher : public image_transport::SimplePublisherPlugin<zdepth_image_transport::msg::ZDepthImage>
#endif
  {
    public:
      virtual ~ZDepthPublisher() {}
      virtual std::string getTransportName() const
      {
        return "zdepth";
      }

    protected:
#if ROS_VERSION_MAJOR == 1
      virtual void publish(const sensor_msgs::Image& img_msg,
                           const PublishFn& publish_fn) const;
#else
      // publish(const Image&, const PublishFn&) is deprecated on ROS2
      // (jazzy), and removed on rolling
      void publish(const sensor_msgs::msg::Image& img_msg,
                   const PublisherT& publisher) const override;
#endif
  };

} //namespace zdepth_image_transport
