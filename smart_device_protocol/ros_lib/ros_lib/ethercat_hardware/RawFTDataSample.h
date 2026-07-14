#ifndef _ROS_ethercat_hardware_RawFTDataSample_h
#define _ROS_ethercat_hardware_RawFTDataSample_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace ethercat_hardware
{

  class RawFTDataSample : public ros::Msg
  {
    public:
      typedef uint64_t _sample_count_type;
      _sample_count_type sample_count;
      uint32_t data_length;
      typedef int16_t _data_type;
      _data_type st_data;
      _data_type * data;
      typedef uint16_t _vhalf_type;
      _vhalf_type vhalf;

    RawFTDataSample():
      sample_count(0),
      data_length(0), st_data(), data(nullptr),
      vhalf(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->sample_count >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->sample_count >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->sample_count >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->sample_count >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (this->sample_count >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (this->sample_count >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (this->sample_count >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (this->sample_count >> (8 * 7)) & 0xFF;
      offset += sizeof(this->sample_count);
      *(outbuffer + offset + 0) = (this->data_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->data_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->data_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->data_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->data_length);
      for( uint32_t i = 0; i < data_length; i++){
      union {
        int16_t real;
        uint16_t base;
      } u_datai;
      u_datai.real = this->data[i];
      *(outbuffer + offset + 0) = (u_datai.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_datai.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->data[i]);
      }
      *(outbuffer + offset + 0) = (this->vhalf >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->vhalf >> (8 * 1)) & 0xFF;
      offset += sizeof(this->vhalf);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      this->sample_count =  ((uint64_t) (*(inbuffer + offset)));
      this->sample_count |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->sample_count |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->sample_count |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->sample_count |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      this->sample_count |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      this->sample_count |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      this->sample_count |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      offset += sizeof(this->sample_count);
      uint32_t data_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      data_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      data_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      data_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->data_length);
      if(data_lengthT > data_length)
        this->data = (int16_t*)realloc(this->data, data_lengthT * sizeof(int16_t));
      data_length = data_lengthT;
      for( uint32_t i = 0; i < data_length; i++){
      union {
        int16_t real;
        uint16_t base;
      } u_st_data;
      u_st_data.base = 0;
      u_st_data.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_data.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->st_data = u_st_data.real;
      offset += sizeof(this->st_data);
        memcpy( &(this->data[i]), &(this->st_data), sizeof(int16_t));
      }
      this->vhalf =  ((uint16_t) (*(inbuffer + offset)));
      this->vhalf |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->vhalf);
     return offset;
    }

    virtual const char * getType() override { return "ethercat_hardware/RawFTDataSample"; };
    virtual const char * getMD5() override { return "6c3b6e352fd24802b2d95b606df80de6"; };

  };

}
#endif
