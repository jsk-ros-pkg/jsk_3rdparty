#ifndef _ROS_SERVICE_CmdVelService_h
#define _ROS_SERVICE_CmdVelService_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "geometry_msgs/Twist.h"

namespace naoqi_bridge_msgs
{

static const char CMDVELSERVICE[] = "naoqi_bridge_msgs/CmdVelService";

  class CmdVelServiceRequest : public ros::Msg
  {
    public:
      typedef geometry_msgs::Twist _twist_type;
      _twist_type twist;

    CmdVelServiceRequest():
      twist()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->twist.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += this->twist.deserialize(inbuffer + offset);
     return offset;
    }

    virtual const char * getType() override { return CMDVELSERVICE; };
    virtual const char * getMD5() override { return "a787b2802160dcc7fe02d089e10afe56"; };

  };

  class CmdVelServiceResponse : public ros::Msg
  {
    public:

    CmdVelServiceResponse()
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

    virtual const char * getType() override { return CMDVELSERVICE; };
    virtual const char * getMD5() override { return "d41d8cd98f00b204e9800998ecf8427e"; };

  };

  class CmdVelService {
    public:
    typedef CmdVelServiceRequest Request;
    typedef CmdVelServiceResponse Response;
  };

}
#endif
