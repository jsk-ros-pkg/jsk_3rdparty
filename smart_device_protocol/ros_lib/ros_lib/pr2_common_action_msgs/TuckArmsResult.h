#ifndef _ROS_pr2_common_action_msgs_TuckArmsResult_h
#define _ROS_pr2_common_action_msgs_TuckArmsResult_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace pr2_common_action_msgs
{

  class TuckArmsResult : public ros::Msg
  {
    public:
      typedef bool _tuck_left_type;
      _tuck_left_type tuck_left;
      typedef bool _tuck_right_type;
      _tuck_right_type tuck_right;

    TuckArmsResult():
      tuck_left(0),
      tuck_right(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_tuck_left;
      u_tuck_left.real = this->tuck_left;
      *(outbuffer + offset + 0) = (u_tuck_left.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->tuck_left);
      union {
        bool real;
        uint8_t base;
      } u_tuck_right;
      u_tuck_right.real = this->tuck_right;
      *(outbuffer + offset + 0) = (u_tuck_right.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->tuck_right);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_tuck_left;
      u_tuck_left.base = 0;
      u_tuck_left.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->tuck_left = u_tuck_left.real;
      offset += sizeof(this->tuck_left);
      union {
        bool real;
        uint8_t base;
      } u_tuck_right;
      u_tuck_right.base = 0;
      u_tuck_right.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->tuck_right = u_tuck_right.real;
      offset += sizeof(this->tuck_right);
     return offset;
    }

    virtual const char * getType() override { return "pr2_common_action_msgs/TuckArmsResult"; };
    virtual const char * getMD5() override { return "a07b11078a50f9881dc3004ca1174834"; };

  };

}
#endif
