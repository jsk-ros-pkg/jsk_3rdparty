#ifndef _ROS_SERVICE_GetTruepose_h
#define _ROS_SERVICE_GetTruepose_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"

namespace naoqi_bridge_msgs
{

static const char GETTRUEPOSE[] = "naoqi_bridge_msgs/GetTruepose";

  class GetTrueposeRequest : public ros::Msg
  {
    public:

    GetTrueposeRequest()
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

    virtual const char * getType() override { return GETTRUEPOSE; };
    virtual const char * getMD5() override { return "d41d8cd98f00b204e9800998ecf8427e"; };

  };

  class GetTrueposeResponse : public ros::Msg
  {
    public:
      typedef geometry_msgs::PoseWithCovarianceStamped _pose_type;
      _pose_type pose;

    GetTrueposeResponse():
      pose()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->pose.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += this->pose.deserialize(inbuffer + offset);
     return offset;
    }

    virtual const char * getType() override { return GETTRUEPOSE; };
    virtual const char * getMD5() override { return "4f3e0bbe7a24e1f929488cd1970222d3"; };

  };

  class GetTruepose {
    public:
    typedef GetTrueposeRequest Request;
    typedef GetTrueposeResponse Response;
  };

}
#endif
