#ifndef _ROS_jsk_gui_msgs_Gravity_h
#define _ROS_jsk_gui_msgs_Gravity_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "geometry_msgs/Vector3.h"

namespace jsk_gui_msgs
{

  class Gravity : public ros::Msg
  {
    public:
      typedef geometry_msgs::Vector3 _gravity_type;
      _gravity_type gravity;

    Gravity():
      gravity()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->gravity.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += this->gravity.deserialize(inbuffer + offset);
     return offset;
    }

    virtual const char * getType() override { return "jsk_gui_msgs/Gravity"; };
    virtual const char * getMD5() override { return "86facaf836997cbbc4faee170616f59e"; };

  };

}
#endif
