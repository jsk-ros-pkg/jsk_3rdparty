#ifndef _ROS_naoqi_bridge_msgs_HandTouch_h
#define _ROS_naoqi_bridge_msgs_HandTouch_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace naoqi_bridge_msgs
{

  class HandTouch : public ros::Msg
  {
    public:
      typedef uint8_t _hand_type;
      _hand_type hand;
      typedef uint8_t _state_type;
      _state_type state;
      enum { RIGHT_BACK = 0 };
      enum { RIGHT_LEFT = 1 };
      enum { RIGHT_RIGHT = 2 };
      enum { LEFT_BACK = 3 };
      enum { LEFT_LEFT = 4 };
      enum { LEFT_RIGHT = 5 };
      enum { stateReleased = 0 };
      enum { statePressed = 1 };

    HandTouch():
      hand(0),
      state(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->hand >> (8 * 0)) & 0xFF;
      offset += sizeof(this->hand);
      *(outbuffer + offset + 0) = (this->state >> (8 * 0)) & 0xFF;
      offset += sizeof(this->state);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      this->hand =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->hand);
      this->state =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->state);
     return offset;
    }

    virtual const char * getType() override { return "naoqi_bridge_msgs/HandTouch"; };
    virtual const char * getMD5() override { return "d8d5c81c96dbe6a9e8e5d80beb70299d"; };

  };

}
#endif
