#ifndef _ROS_SERVICE_BreakerCommand_h
#define _ROS_SERVICE_BreakerCommand_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "power_msgs/BreakerState.h"

namespace power_msgs
{

static const char BREAKERCOMMAND[] = "power_msgs/BreakerCommand";

  class BreakerCommandRequest : public ros::Msg
  {
    public:
      typedef bool _enable_type;
      _enable_type enable;

    BreakerCommandRequest():
      enable(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_enable;
      u_enable.real = this->enable;
      *(outbuffer + offset + 0) = (u_enable.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->enable);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_enable;
      u_enable.base = 0;
      u_enable.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->enable = u_enable.real;
      offset += sizeof(this->enable);
     return offset;
    }

    virtual const char * getType() override { return BREAKERCOMMAND; };
    virtual const char * getMD5() override { return "8c1211af706069c994c06e00eb59ac9e"; };

  };

  class BreakerCommandResponse : public ros::Msg
  {
    public:
      typedef power_msgs::BreakerState _status_type;
      _status_type status;

    BreakerCommandResponse():
      status()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->status.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += this->status.deserialize(inbuffer + offset);
     return offset;
    }

    virtual const char * getType() override { return BREAKERCOMMAND; };
    virtual const char * getMD5() override { return "5cda044252165071cdad9c3098df0594"; };

  };

  class BreakerCommand {
    public:
    typedef BreakerCommandRequest Request;
    typedef BreakerCommandResponse Response;
  };

}
#endif
