#ifndef _ROS_tf2_web_republisher_TFSubscriptionResult_h
#define _ROS_tf2_web_republisher_TFSubscriptionResult_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace tf2_web_republisher
{

  class TFSubscriptionResult : public ros::Msg
  {
    public:

    TFSubscriptionResult()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
     return offset;
    }

    virtual const char * getType() override { return "tf2_web_republisher/TFSubscriptionResult"; };
    virtual const char * getMD5() override { return "d41d8cd98f00b204e9800998ecf8427e"; };

  };

}
#endif
