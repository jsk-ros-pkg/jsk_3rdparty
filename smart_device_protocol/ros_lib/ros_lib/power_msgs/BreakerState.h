#ifndef _ROS_power_msgs_BreakerState_h
#define _ROS_power_msgs_BreakerState_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace power_msgs
{

  class BreakerState : public ros::Msg
  {
    public:
      typedef const char* _name_type;
      _name_type name;
      typedef uint8_t _state_type;
      _state_type state;
      typedef float _current_type;
      _current_type current;
      typedef float _temperature_type;
      _temperature_type temperature;
      typedef float _min_rated_current_type;
      _min_rated_current_type min_rated_current;
      typedef float _max_rated_current_type;
      _max_rated_current_type max_rated_current;
      typedef float _max_rated_temperature_type;
      _max_rated_temperature_type max_rated_temperature;
      enum { STATE_DISABLED =  0 };
      enum { STATE_ENABLED =  1 };
      enum { STATE_ERROR =  2 };

    BreakerState():
      name(""),
      state(0),
      current(0),
      temperature(0),
      min_rated_current(0),
      max_rated_current(0),
      max_rated_temperature(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      uint32_t length_name = strlen(this->name);
      varToArr(outbuffer + offset, length_name);
      offset += 4;
      memcpy(outbuffer + offset, this->name, length_name);
      offset += length_name;
      *(outbuffer + offset + 0) = (this->state >> (8 * 0)) & 0xFF;
      offset += sizeof(this->state);
      union {
        float real;
        uint32_t base;
      } u_current;
      u_current.real = this->current;
      *(outbuffer + offset + 0) = (u_current.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_current.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_current.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_current.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->current);
      union {
        float real;
        uint32_t base;
      } u_temperature;
      u_temperature.real = this->temperature;
      *(outbuffer + offset + 0) = (u_temperature.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_temperature.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_temperature.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_temperature.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->temperature);
      union {
        float real;
        uint32_t base;
      } u_min_rated_current;
      u_min_rated_current.real = this->min_rated_current;
      *(outbuffer + offset + 0) = (u_min_rated_current.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_min_rated_current.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_min_rated_current.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_min_rated_current.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->min_rated_current);
      union {
        float real;
        uint32_t base;
      } u_max_rated_current;
      u_max_rated_current.real = this->max_rated_current;
      *(outbuffer + offset + 0) = (u_max_rated_current.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_max_rated_current.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_max_rated_current.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_max_rated_current.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->max_rated_current);
      union {
        float real;
        uint32_t base;
      } u_max_rated_temperature;
      u_max_rated_temperature.real = this->max_rated_temperature;
      *(outbuffer + offset + 0) = (u_max_rated_temperature.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_max_rated_temperature.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_max_rated_temperature.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_max_rated_temperature.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->max_rated_temperature);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      uint32_t length_name;
      arrToVar(length_name, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_name; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_name-1]=0;
      this->name = (char *)(inbuffer + offset-1);
      offset += length_name;
      this->state =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->state);
      union {
        float real;
        uint32_t base;
      } u_current;
      u_current.base = 0;
      u_current.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_current.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_current.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_current.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->current = u_current.real;
      offset += sizeof(this->current);
      union {
        float real;
        uint32_t base;
      } u_temperature;
      u_temperature.base = 0;
      u_temperature.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_temperature.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_temperature.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_temperature.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->temperature = u_temperature.real;
      offset += sizeof(this->temperature);
      union {
        float real;
        uint32_t base;
      } u_min_rated_current;
      u_min_rated_current.base = 0;
      u_min_rated_current.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_min_rated_current.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_min_rated_current.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_min_rated_current.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->min_rated_current = u_min_rated_current.real;
      offset += sizeof(this->min_rated_current);
      union {
        float real;
        uint32_t base;
      } u_max_rated_current;
      u_max_rated_current.base = 0;
      u_max_rated_current.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_max_rated_current.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_max_rated_current.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_max_rated_current.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->max_rated_current = u_max_rated_current.real;
      offset += sizeof(this->max_rated_current);
      union {
        float real;
        uint32_t base;
      } u_max_rated_temperature;
      u_max_rated_temperature.base = 0;
      u_max_rated_temperature.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_max_rated_temperature.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_max_rated_temperature.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_max_rated_temperature.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->max_rated_temperature = u_max_rated_temperature.real;
      offset += sizeof(this->max_rated_temperature);
     return offset;
    }

    virtual const char * getType() override { return "power_msgs/BreakerState"; };
    virtual const char * getMD5() override { return "e8cf206acdcddee3412681363f829642"; };

  };

}
#endif
