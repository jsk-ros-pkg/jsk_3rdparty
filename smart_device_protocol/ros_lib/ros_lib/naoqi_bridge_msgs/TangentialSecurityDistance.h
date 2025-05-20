#ifndef _ROS_SERVICE_TangentialSecurityDistance_h
#define _ROS_SERVICE_TangentialSecurityDistance_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Float32.h"

namespace naoqi_bridge_msgs
{

static const char TANGENTIALSECURITYDISTANCE[] = "naoqi_bridge_msgs/TangentialSecurityDistance";

  class TangentialSecurityDistanceRequest : public ros::Msg
  {
    public:
      typedef std_msgs::Float32 _tangential_distance_type;
      _tangential_distance_type tangential_distance;

    TangentialSecurityDistanceRequest():
      tangential_distance()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->tangential_distance.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += this->tangential_distance.deserialize(inbuffer + offset);
     return offset;
    }

    virtual const char * getType() override { return TANGENTIALSECURITYDISTANCE; };
    virtual const char * getMD5() override { return "b07653f2626a354d4219619fffc76403"; };

  };

  class TangentialSecurityDistanceResponse : public ros::Msg
  {
    public:

    TangentialSecurityDistanceResponse()
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

    virtual const char * getType() override { return TANGENTIALSECURITYDISTANCE; };
    virtual const char * getMD5() override { return "d41d8cd98f00b204e9800998ecf8427e"; };

  };

  class TangentialSecurityDistance {
    public:
    typedef TangentialSecurityDistanceRequest Request;
    typedef TangentialSecurityDistanceResponse Response;
  };

}
#endif
