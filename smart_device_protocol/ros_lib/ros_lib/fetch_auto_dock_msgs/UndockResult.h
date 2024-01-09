#ifndef _ROS_fetch_auto_dock_msgs_UndockResult_h
#define _ROS_fetch_auto_dock_msgs_UndockResult_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace fetch_auto_dock_msgs
{

  class UndockResult : public ros::Msg
  {
    public:
      typedef bool _undocked_type;
      _undocked_type undocked;

    UndockResult():
      undocked(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_undocked;
      u_undocked.real = this->undocked;
      *(outbuffer + offset + 0) = (u_undocked.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->undocked);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_undocked;
      u_undocked.base = 0;
      u_undocked.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->undocked = u_undocked.real;
      offset += sizeof(this->undocked);
     return offset;
    }

    virtual const char * getType() override { return "fetch_auto_dock_msgs/UndockResult"; };
    virtual const char * getMD5() override { return "26f25fb4c0c6a6805fac81b37bb7b9c9"; };

  };

}
#endif
