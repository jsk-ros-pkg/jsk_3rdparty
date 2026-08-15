#ifndef _ROS_robot_calibration_msgs_GripperLedCommandGoal_h
#define _ROS_robot_calibration_msgs_GripperLedCommandGoal_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace robot_calibration_msgs
{

  class GripperLedCommandGoal : public ros::Msg
  {
    public:
      typedef uint8_t _led_code_type;
      _led_code_type led_code;

    GripperLedCommandGoal():
      led_code(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->led_code >> (8 * 0)) & 0xFF;
      offset += sizeof(this->led_code);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      this->led_code =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->led_code);
     return offset;
    }

    virtual const char * getType() override { return "robot_calibration_msgs/GripperLedCommandGoal"; };
    virtual const char * getMD5() override { return "11513917910062a2ca72ae2699b68556"; };

  };

}
#endif
