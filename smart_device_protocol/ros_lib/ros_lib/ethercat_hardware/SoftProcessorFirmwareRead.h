#ifndef _ROS_SERVICE_SoftProcessorFirmwareRead_h
#define _ROS_SERVICE_SoftProcessorFirmwareRead_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace ethercat_hardware
{

static const char SOFTPROCESSORFIRMWAREREAD[] = "ethercat_hardware/SoftProcessorFirmwareRead";

  class SoftProcessorFirmwareReadRequest : public ros::Msg
  {
    public:
      typedef const char* _actuator_name_type;
      _actuator_name_type actuator_name;
      typedef const char* _processor_name_type;
      _processor_name_type processor_name;

    SoftProcessorFirmwareReadRequest():
      actuator_name(""),
      processor_name("")
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      uint32_t length_actuator_name = strlen(this->actuator_name);
      varToArr(outbuffer + offset, length_actuator_name);
      offset += 4;
      memcpy(outbuffer + offset, this->actuator_name, length_actuator_name);
      offset += length_actuator_name;
      uint32_t length_processor_name = strlen(this->processor_name);
      varToArr(outbuffer + offset, length_processor_name);
      offset += 4;
      memcpy(outbuffer + offset, this->processor_name, length_processor_name);
      offset += length_processor_name;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      uint32_t length_actuator_name;
      arrToVar(length_actuator_name, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_actuator_name; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_actuator_name-1]=0;
      this->actuator_name = (char *)(inbuffer + offset-1);
      offset += length_actuator_name;
      uint32_t length_processor_name;
      arrToVar(length_processor_name, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_processor_name; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_processor_name-1]=0;
      this->processor_name = (char *)(inbuffer + offset-1);
      offset += length_processor_name;
     return offset;
    }

    virtual const char * getType() override { return SOFTPROCESSORFIRMWAREREAD; };
    virtual const char * getMD5() override { return "777be25d71e9e85e62fa14223ffddb6b"; };

  };

  class SoftProcessorFirmwareReadResponse : public ros::Msg
  {
    public:
      typedef bool _success_type;
      _success_type success;
      typedef const char* _error_msg_type;
      _error_msg_type error_msg;
      uint32_t instructions_length;
      typedef uint32_t _instructions_type;
      _instructions_type st_instructions;
      _instructions_type * instructions;

    SoftProcessorFirmwareReadResponse():
      success(0),
      error_msg(""),
      instructions_length(0), st_instructions(), instructions(nullptr)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_success;
      u_success.real = this->success;
      *(outbuffer + offset + 0) = (u_success.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->success);
      uint32_t length_error_msg = strlen(this->error_msg);
      varToArr(outbuffer + offset, length_error_msg);
      offset += 4;
      memcpy(outbuffer + offset, this->error_msg, length_error_msg);
      offset += length_error_msg;
      *(outbuffer + offset + 0) = (this->instructions_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->instructions_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->instructions_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->instructions_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->instructions_length);
      for( uint32_t i = 0; i < instructions_length; i++){
      *(outbuffer + offset + 0) = (this->instructions[i] >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->instructions[i] >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->instructions[i] >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->instructions[i] >> (8 * 3)) & 0xFF;
      offset += sizeof(this->instructions[i]);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_success;
      u_success.base = 0;
      u_success.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->success = u_success.real;
      offset += sizeof(this->success);
      uint32_t length_error_msg;
      arrToVar(length_error_msg, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_error_msg; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_error_msg-1]=0;
      this->error_msg = (char *)(inbuffer + offset-1);
      offset += length_error_msg;
      uint32_t instructions_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      instructions_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      instructions_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      instructions_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->instructions_length);
      if(instructions_lengthT > instructions_length)
        this->instructions = (uint32_t*)realloc(this->instructions, instructions_lengthT * sizeof(uint32_t));
      instructions_length = instructions_lengthT;
      for( uint32_t i = 0; i < instructions_length; i++){
      this->st_instructions =  ((uint32_t) (*(inbuffer + offset)));
      this->st_instructions |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->st_instructions |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->st_instructions |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->st_instructions);
        memcpy( &(this->instructions[i]), &(this->st_instructions), sizeof(uint32_t));
      }
     return offset;
    }

    virtual const char * getType() override { return SOFTPROCESSORFIRMWAREREAD; };
    virtual const char * getMD5() override { return "d36ea5e74d6ac75d45ab5ae553b4d4e6"; };

  };

  class SoftProcessorFirmwareRead {
    public:
    typedef SoftProcessorFirmwareReadRequest Request;
    typedef SoftProcessorFirmwareReadResponse Response;
  };

}
#endif
