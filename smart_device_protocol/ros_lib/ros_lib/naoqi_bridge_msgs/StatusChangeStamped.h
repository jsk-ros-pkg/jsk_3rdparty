#ifndef _ROS_naoqi_bridge_msgs_StatusChangeStamped_h
#define _ROS_naoqi_bridge_msgs_StatusChangeStamped_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "std_msgs/String.h"

namespace naoqi_bridge_msgs
{

  class StatusChangeStamped : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef int16_t _recharge_type_type;
      _recharge_type_type recharge_type;
      typedef std_msgs::String _old_status_type;
      _old_status_type old_status;
      typedef std_msgs::String _new_status_type;
      _new_status_type new_status;

    StatusChangeStamped():
      header(),
      recharge_type(0),
      old_status(),
      new_status()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      union {
        int16_t real;
        uint16_t base;
      } u_recharge_type;
      u_recharge_type.real = this->recharge_type;
      *(outbuffer + offset + 0) = (u_recharge_type.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_recharge_type.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->recharge_type);
      offset += this->old_status.serialize(outbuffer + offset);
      offset += this->new_status.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      union {
        int16_t real;
        uint16_t base;
      } u_recharge_type;
      u_recharge_type.base = 0;
      u_recharge_type.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_recharge_type.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->recharge_type = u_recharge_type.real;
      offset += sizeof(this->recharge_type);
      offset += this->old_status.deserialize(inbuffer + offset);
      offset += this->new_status.deserialize(inbuffer + offset);
     return offset;
    }

    virtual const char * getType() override { return "naoqi_bridge_msgs/StatusChangeStamped"; };
    virtual const char * getMD5() override { return "631ab2246eca82d839e3a99b76567775"; };

  };

}
#endif
