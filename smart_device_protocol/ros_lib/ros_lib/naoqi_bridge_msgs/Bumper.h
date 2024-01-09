#ifndef _ROS_naoqi_bridge_msgs_Bumper_h
#define _ROS_naoqi_bridge_msgs_Bumper_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace naoqi_bridge_msgs
{

  class Bumper : public ros::Msg
  {
    public:
      typedef uint8_t _bumper_type;
      _bumper_type bumper;
      typedef uint8_t _state_type;
      _state_type state;
      enum { right = 0 };
      enum { left = 1 };
      enum { back = 2 };
      enum { stateReleased = 0 };
      enum { statePressed = 1 };

    Bumper():
      bumper(0),
      state(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->bumper >> (8 * 0)) & 0xFF;
      offset += sizeof(this->bumper);
      *(outbuffer + offset + 0) = (this->state >> (8 * 0)) & 0xFF;
      offset += sizeof(this->state);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      this->bumper =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->bumper);
      this->state =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->state);
     return offset;
    }

    virtual const char * getType() override { return "naoqi_bridge_msgs/Bumper"; };
    virtual const char * getMD5() override { return "89965a81ab868825f18d59365e28ddaf"; };

  };

}
#endif
