#include "zdepth_image_transport/ros_version.h"
#if ROS_VERSION_MAJOR == 1
#include <image_transport/simple_subscriber_plugin.h>
#else
#include <image_transport/simple_subscriber_plugin.hpp>
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
  class ZDepthSubscriber : public image_transport::SimpleSubscriberPlugin<zdepth_image_transport::ZDepthImage>
#else
  class ZDepthSubscriber : public image_transport::SimpleSubscriberPlugin<zdepth_image_transport::msg::ZDepthImage>
#endif
  {
    public:
      virtual ~ZDepthSubscriber() {}
      virtual std::string getTransportName() const
      {
        return "zdepth";
      }

    protected:
#if ROS_VERSION_MAJOR == 1
      virtual void internalCallback(const zdepth_image_transport::ZDepthImageConstPtr& zdepth_msg,
                                    const Callback& user_cb);
#else
      void internalCallback(const zdepth_image_transport::msg::ZDepthImage::ConstSharedPtr& zdepth_msg,
                            const Callback& user_cb) override;
#endif
  };

} //namespace zdepth_image_transport
