#ifndef _ROS_SERVICE_GetRobotInfo_h
#define _ROS_SERVICE_GetRobotInfo_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "naoqi_bridge_msgs/RobotInfo.h"

namespace naoqi_bridge_msgs
{

static const char GETROBOTINFO[] = "naoqi_bridge_msgs/GetRobotInfo";

  class GetRobotInfoRequest : public ros::Msg
  {
    public:

    GetRobotInfoRequest()
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

    virtual const char * getType() override { return GETROBOTINFO; };
    virtual const char * getMD5() override { return "d41d8cd98f00b204e9800998ecf8427e"; };

  };

  class GetRobotInfoResponse : public ros::Msg
  {
    public:
      typedef naoqi_bridge_msgs::RobotInfo _info_type;
      _info_type info;

    GetRobotInfoResponse():
      info()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->info.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += this->info.deserialize(inbuffer + offset);
     return offset;
    }

    virtual const char * getType() override { return GETROBOTINFO; };
    virtual const char * getMD5() override { return "6a705e4ea65692d6e21188b3537da43d"; };

  };

  class GetRobotInfo {
    public:
    typedef GetRobotInfoRequest Request;
    typedef GetRobotInfoResponse Response;
  };

}
#endif
