#ifndef _ROS_naoqi_bridge_msgs_HeadTouch_h
#define _ROS_naoqi_bridge_msgs_HeadTouch_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace naoqi_bridge_msgs
{

  class HeadTouch : public ros::Msg
  {
    public:
      typedef uint8_t _button_type;
      _button_type button;
      typedef uint8_t _state_type;
      _state_type state;
      enum { buttonFront = 1 };
      enum { buttonMiddle = 2 };
      enum { buttonRear = 3 };
      enum { stateReleased = 0 };
      enum { statePressed = 1 };

    HeadTouch():
      button(0),
      state(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->button >> (8 * 0)) & 0xFF;
      offset += sizeof(this->button);
      *(outbuffer + offset + 0) = (this->state >> (8 * 0)) & 0xFF;
      offset += sizeof(this->state);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      this->button =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->button);
      this->state =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->state);
     return offset;
    }

    virtual const char * getType() override { return "naoqi_bridge_msgs/HeadTouch"; };
    virtual const char * getMD5() override { return "b75165bf9dfed26d50ad4e3162304225"; };

  };

}
#endif
