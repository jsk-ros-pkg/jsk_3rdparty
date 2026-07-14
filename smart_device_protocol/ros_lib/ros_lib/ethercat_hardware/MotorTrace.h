#ifndef _ROS_ethercat_hardware_MotorTrace_h
#define _ROS_ethercat_hardware_MotorTrace_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "ethercat_hardware/BoardInfo.h"
#include "ethercat_hardware/ActuatorInfo.h"
#include "ethercat_hardware/MotorTraceSample.h"

namespace ethercat_hardware
{

  class MotorTrace : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef const char* _reason_type;
      _reason_type reason;
      typedef ethercat_hardware::BoardInfo _board_info_type;
      _board_info_type board_info;
      typedef ethercat_hardware::ActuatorInfo _actuator_info_type;
      _actuator_info_type actuator_info;
      uint32_t samples_length;
      typedef ethercat_hardware::MotorTraceSample _samples_type;
      _samples_type st_samples;
      _samples_type * samples;

    MotorTrace():
      header(),
      reason(""),
      board_info(),
      actuator_info(),
      samples_length(0), st_samples(), samples(nullptr)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      uint32_t length_reason = strlen(this->reason);
      varToArr(outbuffer + offset, length_reason);
      offset += 4;
      memcpy(outbuffer + offset, this->reason, length_reason);
      offset += length_reason;
      offset += this->board_info.serialize(outbuffer + offset);
      offset += this->actuator_info.serialize(outbuffer + offset);
      *(outbuffer + offset + 0) = (this->samples_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->samples_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->samples_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->samples_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->samples_length);
      for( uint32_t i = 0; i < samples_length; i++){
      offset += this->samples[i].serialize(outbuffer + offset);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      uint32_t length_reason;
      arrToVar(length_reason, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_reason; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_reason-1]=0;
      this->reason = (char *)(inbuffer + offset-1);
      offset += length_reason;
      offset += this->board_info.deserialize(inbuffer + offset);
      offset += this->actuator_info.deserialize(inbuffer + offset);
      uint32_t samples_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      samples_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      samples_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      samples_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->samples_length);
      if(samples_lengthT > samples_length)
        this->samples = (ethercat_hardware::MotorTraceSample*)realloc(this->samples, samples_lengthT * sizeof(ethercat_hardware::MotorTraceSample));
      samples_length = samples_lengthT;
      for( uint32_t i = 0; i < samples_length; i++){
      offset += this->st_samples.deserialize(inbuffer + offset);
        memcpy( &(this->samples[i]), &(this->st_samples), sizeof(ethercat_hardware::MotorTraceSample));
      }
     return offset;
    }

    virtual const char * getType() override { return "ethercat_hardware/MotorTrace"; };
    virtual const char * getMD5() override { return "ada0b8b7f00967d292bd5bb4f59d4bd8"; };

  };

}
#endif
