#ifndef _ROS_pr2_msgs_PowerState_h
#define _ROS_pr2_msgs_PowerState_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "ros/duration.h"

namespace pr2_msgs
{

  class PowerState : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef float _power_consumption_type;
      _power_consumption_type power_consumption;
      typedef ros::Duration _time_remaining_type;
      _time_remaining_type time_remaining;
      typedef const char* _prediction_method_type;
      _prediction_method_type prediction_method;
      typedef int8_t _relative_capacity_type;
      _relative_capacity_type relative_capacity;
      typedef int8_t _AC_present_type;
      _AC_present_type AC_present;

    PowerState():
      header(),
      power_consumption(0),
      time_remaining(),
      prediction_method(""),
      relative_capacity(0),
      AC_present(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      offset += serializeAvrFloat64(outbuffer + offset, this->power_consumption);
      *(outbuffer + offset + 0) = (this->time_remaining.sec >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->time_remaining.sec >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->time_remaining.sec >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->time_remaining.sec >> (8 * 3)) & 0xFF;
      offset += sizeof(this->time_remaining.sec);
      *(outbuffer + offset + 0) = (this->time_remaining.nsec >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->time_remaining.nsec >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->time_remaining.nsec >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->time_remaining.nsec >> (8 * 3)) & 0xFF;
      offset += sizeof(this->time_remaining.nsec);
      uint32_t length_prediction_method = strlen(this->prediction_method);
      varToArr(outbuffer + offset, length_prediction_method);
      offset += 4;
      memcpy(outbuffer + offset, this->prediction_method, length_prediction_method);
      offset += length_prediction_method;
      union {
        int8_t real;
        uint8_t base;
      } u_relative_capacity;
      u_relative_capacity.real = this->relative_capacity;
      *(outbuffer + offset + 0) = (u_relative_capacity.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->relative_capacity);
      union {
        int8_t real;
        uint8_t base;
      } u_AC_present;
      u_AC_present.real = this->AC_present;
      *(outbuffer + offset + 0) = (u_AC_present.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->AC_present);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->power_consumption));
      this->time_remaining.sec =  ((uint32_t) (*(inbuffer + offset)));
      this->time_remaining.sec |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->time_remaining.sec |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->time_remaining.sec |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->time_remaining.sec);
      this->time_remaining.nsec =  ((uint32_t) (*(inbuffer + offset)));
      this->time_remaining.nsec |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->time_remaining.nsec |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->time_remaining.nsec |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->time_remaining.nsec);
      uint32_t length_prediction_method;
      arrToVar(length_prediction_method, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_prediction_method; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_prediction_method-1]=0;
      this->prediction_method = (char *)(inbuffer + offset-1);
      offset += length_prediction_method;
      union {
        int8_t real;
        uint8_t base;
      } u_relative_capacity;
      u_relative_capacity.base = 0;
      u_relative_capacity.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->relative_capacity = u_relative_capacity.real;
      offset += sizeof(this->relative_capacity);
      union {
        int8_t real;
        uint8_t base;
      } u_AC_present;
      u_AC_present.base = 0;
      u_AC_present.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->AC_present = u_AC_present.real;
      offset += sizeof(this->AC_present);
     return offset;
    }

    virtual const char * getType() override { return "pr2_msgs/PowerState"; };
    virtual const char * getMD5() override { return "e6fa46a387cad0b7a80959a21587a6c9"; };

  };

}
#endif
