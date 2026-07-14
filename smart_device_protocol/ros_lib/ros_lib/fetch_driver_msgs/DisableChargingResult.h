#ifndef _ROS_fetch_driver_msgs_DisableChargingResult_h
#define _ROS_fetch_driver_msgs_DisableChargingResult_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace fetch_driver_msgs
{

  class DisableChargingResult : public ros::Msg
  {
    public:
      typedef bool _charging_disabled_type;
      _charging_disabled_type charging_disabled;

    DisableChargingResult():
      charging_disabled(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_charging_disabled;
      u_charging_disabled.real = this->charging_disabled;
      *(outbuffer + offset + 0) = (u_charging_disabled.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->charging_disabled);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_charging_disabled;
      u_charging_disabled.base = 0;
      u_charging_disabled.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->charging_disabled = u_charging_disabled.real;
      offset += sizeof(this->charging_disabled);
     return offset;
    }

    virtual const char * getType() override { return "fetch_driver_msgs/DisableChargingResult"; };
    virtual const char * getMD5() override { return "27dccd3ed9b50034b06448bfbba88780"; };

  };

}
#endif
