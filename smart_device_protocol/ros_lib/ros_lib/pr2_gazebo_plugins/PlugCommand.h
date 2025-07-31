#ifndef _ROS_pr2_gazebo_plugins_PlugCommand_h
#define _ROS_pr2_gazebo_plugins_PlugCommand_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace pr2_gazebo_plugins
{

  class PlugCommand : public ros::Msg
  {
    public:
      typedef bool _ac_present_type;
      _ac_present_type ac_present;
      typedef float _charge_rate_type;
      _charge_rate_type charge_rate;
      typedef float _discharge_rate_type;
      _discharge_rate_type discharge_rate;
      typedef float _charge_type;
      _charge_type charge;

    PlugCommand():
      ac_present(0),
      charge_rate(0),
      discharge_rate(0),
      charge(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_ac_present;
      u_ac_present.real = this->ac_present;
      *(outbuffer + offset + 0) = (u_ac_present.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->ac_present);
      offset += serializeAvrFloat64(outbuffer + offset, this->charge_rate);
      offset += serializeAvrFloat64(outbuffer + offset, this->discharge_rate);
      offset += serializeAvrFloat64(outbuffer + offset, this->charge);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_ac_present;
      u_ac_present.base = 0;
      u_ac_present.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->ac_present = u_ac_present.real;
      offset += sizeof(this->ac_present);
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->charge_rate));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->discharge_rate));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->charge));
     return offset;
    }

    virtual const char * getType() override { return "pr2_gazebo_plugins/PlugCommand"; };
    virtual const char * getMD5() override { return "852b7035ee3e7fa6390824cf7b7e6dd1"; };

  };

}
#endif
