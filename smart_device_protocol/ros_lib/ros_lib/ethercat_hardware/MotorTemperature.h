#ifndef _ROS_ethercat_hardware_MotorTemperature_h
#define _ROS_ethercat_hardware_MotorTemperature_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "ros/time.h"

namespace ethercat_hardware
{

  class MotorTemperature : public ros::Msg
  {
    public:
      typedef ros::Time _stamp_type;
      _stamp_type stamp;
      typedef float _winding_temperature_type;
      _winding_temperature_type winding_temperature;
      typedef float _housing_temperature_type;
      _housing_temperature_type housing_temperature;
      typedef float _ambient_temperature_type;
      _ambient_temperature_type ambient_temperature;
      typedef float _heating_power_type;
      _heating_power_type heating_power;

    MotorTemperature():
      stamp(),
      winding_temperature(0),
      housing_temperature(0),
      ambient_temperature(0),
      heating_power(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->stamp.sec >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->stamp.sec >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->stamp.sec >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->stamp.sec >> (8 * 3)) & 0xFF;
      offset += sizeof(this->stamp.sec);
      *(outbuffer + offset + 0) = (this->stamp.nsec >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->stamp.nsec >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->stamp.nsec >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->stamp.nsec >> (8 * 3)) & 0xFF;
      offset += sizeof(this->stamp.nsec);
      offset += serializeAvrFloat64(outbuffer + offset, this->winding_temperature);
      offset += serializeAvrFloat64(outbuffer + offset, this->housing_temperature);
      offset += serializeAvrFloat64(outbuffer + offset, this->ambient_temperature);
      offset += serializeAvrFloat64(outbuffer + offset, this->heating_power);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      this->stamp.sec =  ((uint32_t) (*(inbuffer + offset)));
      this->stamp.sec |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->stamp.sec |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->stamp.sec |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->stamp.sec);
      this->stamp.nsec =  ((uint32_t) (*(inbuffer + offset)));
      this->stamp.nsec |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->stamp.nsec |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->stamp.nsec |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->stamp.nsec);
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->winding_temperature));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->housing_temperature));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->ambient_temperature));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->heating_power));
     return offset;
    }

    virtual const char * getType() override { return "ethercat_hardware/MotorTemperature"; };
    virtual const char * getMD5() override { return "d8c7239cd096d6f25b75bff6b63f2162"; };

  };

}
#endif
