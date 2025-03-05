#include <image_transport/simple_subscriber_plugin.h>
#include "zdepth/zdepth.hpp"
#include "zdepth_image_transport/ZDepthImage.h"


namespace zdepth_image_transport
{
  class ZDepthSubscriber : public image_transport::SimpleSubscriberPlugin<zdepth_image_transport::ZDepthImage>
  {
    public:
      virtual ~ZDepthSubscriber() {}
      virtual std::string getTransportName() const
      {
        return "zdepth";
      }

    protected:
      virtual void internalCallback(const zdepth_image_transport::ZDepthImageConstPtr& zdepth_msg,
                                    const Callback& user_cb);
  };

} //namespace zdepth_image_transport
