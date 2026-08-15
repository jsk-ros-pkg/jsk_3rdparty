#ifndef _ROS_SERVICE_SetTransform_h
#define _ROS_SERVICE_SetTransform_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "geometry_msgs/Transform.h"

namespace naoqi_bridge_msgs
{

static const char SETTRANSFORM[] = "naoqi_bridge_msgs/SetTransform";

  class SetTransformRequest : public ros::Msg
  {
    public:
      typedef geometry_msgs::Transform _offset_type;
      _offset_type offset;

    SetTransformRequest():
      offset()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->offset.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += this->offset.deserialize(inbuffer + offset);
     return offset;
    }

    virtual const char * getType() override { return SETTRANSFORM; };
    virtual const char * getMD5() override { return "67035ddf415a9bb64191f0e45b060e35"; };

  };

  class SetTransformResponse : public ros::Msg
  {
    public:

    SetTransformResponse()
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

    virtual const char * getType() override { return SETTRANSFORM; };
    virtual const char * getMD5() override { return "d41d8cd98f00b204e9800998ecf8427e"; };

  };

  class SetTransform {
    public:
    typedef SetTransformRequest Request;
    typedef SetTransformResponse Response;
  };

}
#endif
