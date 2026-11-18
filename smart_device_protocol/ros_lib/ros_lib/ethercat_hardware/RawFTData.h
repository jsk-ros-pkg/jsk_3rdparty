#ifndef _ROS_ethercat_hardware_RawFTData_h
#define _ROS_ethercat_hardware_RawFTData_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "ethercat_hardware/RawFTDataSample.h"

namespace ethercat_hardware
{

  class RawFTData : public ros::Msg
  {
    public:
      uint32_t samples_length;
      typedef ethercat_hardware::RawFTDataSample _samples_type;
      _samples_type st_samples;
      _samples_type * samples;
      typedef int64_t _sample_count_type;
      _sample_count_type sample_count;
      typedef int64_t _missed_samples_type;
      _missed_samples_type missed_samples;

    RawFTData():
      samples_length(0), st_samples(), samples(nullptr),
      sample_count(0),
      missed_samples(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->samples_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->samples_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->samples_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->samples_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->samples_length);
      for( uint32_t i = 0; i < samples_length; i++){
      offset += this->samples[i].serialize(outbuffer + offset);
      }
      union {
        int64_t real;
        uint64_t base;
      } u_sample_count;
      u_sample_count.real = this->sample_count;
      *(outbuffer + offset + 0) = (u_sample_count.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_sample_count.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_sample_count.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_sample_count.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_sample_count.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_sample_count.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_sample_count.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_sample_count.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->sample_count);
      union {
        int64_t real;
        uint64_t base;
      } u_missed_samples;
      u_missed_samples.real = this->missed_samples;
      *(outbuffer + offset + 0) = (u_missed_samples.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_missed_samples.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_missed_samples.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_missed_samples.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_missed_samples.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_missed_samples.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_missed_samples.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_missed_samples.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->missed_samples);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      uint32_t samples_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      samples_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      samples_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      samples_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->samples_length);
      if(samples_lengthT > samples_length)
        this->samples = (ethercat_hardware::RawFTDataSample*)realloc(this->samples, samples_lengthT * sizeof(ethercat_hardware::RawFTDataSample));
      samples_length = samples_lengthT;
      for( uint32_t i = 0; i < samples_length; i++){
      offset += this->st_samples.deserialize(inbuffer + offset);
        memcpy( &(this->samples[i]), &(this->st_samples), sizeof(ethercat_hardware::RawFTDataSample));
      }
      union {
        int64_t real;
        uint64_t base;
      } u_sample_count;
      u_sample_count.base = 0;
      u_sample_count.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_sample_count.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_sample_count.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_sample_count.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_sample_count.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_sample_count.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_sample_count.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_sample_count.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->sample_count = u_sample_count.real;
      offset += sizeof(this->sample_count);
      union {
        int64_t real;
        uint64_t base;
      } u_missed_samples;
      u_missed_samples.base = 0;
      u_missed_samples.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_missed_samples.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_missed_samples.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_missed_samples.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_missed_samples.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_missed_samples.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_missed_samples.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_missed_samples.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->missed_samples = u_missed_samples.real;
      offset += sizeof(this->missed_samples);
     return offset;
    }

    virtual const char * getType() override { return "ethercat_hardware/RawFTData"; };
    virtual const char * getMD5() override { return "85f5ed45095367bfb8fb2e57954c0b89"; };

  };

}
#endif
