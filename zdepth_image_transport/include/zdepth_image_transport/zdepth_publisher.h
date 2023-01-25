#include <image_transport/simple_publisher_plugin.h>
#include "zdepth/zdepth.hpp"
#include "zdepth_image_transport/ZDepthImage.h"


namespace zdepth_image_transport
{
  class ZDepthPublisher : public image_transport::SimplePublisherPlugin<zdepth_image_transport::ZDepthImage>
  {
    public:
      virtual ~ZDepthPublisher() {}
      virtual std::string getTransportName() const
      {
        return "zdepth";
      }

    protected:
      virtual void publish(const sensor_msgs::Image& img_msg,
                           const PublishFn& publish_fn) const;
  };

} //namespace zdepth_image_transport
