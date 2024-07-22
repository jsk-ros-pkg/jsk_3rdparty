#ifndef _ROS_fingertip_pressure_PressureInfo_h
#define _ROS_fingertip_pressure_PressureInfo_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "fingertip_pressure/PressureInfoElement.h"

namespace fingertip_pressure
{

  class PressureInfo : public ros::Msg
  {
    public:
      uint32_t sensor_length;
      typedef fingertip_pressure::PressureInfoElement _sensor_type;
      _sensor_type st_sensor;
      _sensor_type * sensor;

    PressureInfo():
      sensor_length(0), st_sensor(), sensor(nullptr)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->sensor_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->sensor_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->sensor_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->sensor_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->sensor_length);
      for( uint32_t i = 0; i < sensor_length; i++){
      offset += this->sensor[i].serialize(outbuffer + offset);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      uint32_t sensor_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      sensor_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      sensor_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      sensor_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->sensor_length);
      if(sensor_lengthT > sensor_length)
        this->sensor = (fingertip_pressure::PressureInfoElement*)realloc(this->sensor, sensor_lengthT * sizeof(fingertip_pressure::PressureInfoElement));
      sensor_length = sensor_lengthT;
      for( uint32_t i = 0; i < sensor_length; i++){
      offset += this->st_sensor.deserialize(inbuffer + offset);
        memcpy( &(this->sensor[i]), &(this->st_sensor), sizeof(fingertip_pressure::PressureInfoElement));
      }
     return offset;
    }

    virtual const char * getType() override { return "fingertip_pressure/PressureInfo"; };
    virtual const char * getMD5() override { return "a11fc5bae3534aa023741e378743af5b"; };

  };

}
#endif
