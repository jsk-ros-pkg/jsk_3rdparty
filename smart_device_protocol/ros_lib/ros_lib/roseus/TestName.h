#ifndef _ROS_roseus_TestName_h
#define _ROS_roseus_TestName_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "roseus/StringStamped.h"

namespace roseus
{

  class TestName : public ros::Msg
  {
    public:
      typedef roseus::StringStamped _name_type;
      _name_type name;

    TestName():
      name()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->name.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += this->name.deserialize(inbuffer + offset);
     return offset;
    }

    virtual const char * getType() override { return "roseus/TestName"; };
    virtual const char * getMD5() override { return "70bc7fd92cd8428f6a02d7d0df4d9b80"; };

  };

}
#endif
