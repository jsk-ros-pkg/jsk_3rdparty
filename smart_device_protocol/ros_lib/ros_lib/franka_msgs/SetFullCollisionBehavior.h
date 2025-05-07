#ifndef _ROS_SERVICE_SetFullCollisionBehavior_h
#define _ROS_SERVICE_SetFullCollisionBehavior_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace franka_msgs
{

static const char SETFULLCOLLISIONBEHAVIOR[] = "franka_msgs/SetFullCollisionBehavior";

  class SetFullCollisionBehaviorRequest : public ros::Msg
  {
    public:
      float lower_torque_thresholds_acceleration[7];
      float upper_torque_thresholds_acceleration[7];
      float lower_torque_thresholds_nominal[7];
      float upper_torque_thresholds_nominal[7];
      float lower_force_thresholds_acceleration[6];
      float upper_force_thresholds_acceleration[6];
      float lower_force_thresholds_nominal[6];
      float upper_force_thresholds_nominal[6];

    SetFullCollisionBehaviorRequest():
      lower_torque_thresholds_acceleration(),
      upper_torque_thresholds_acceleration(),
      lower_torque_thresholds_nominal(),
      upper_torque_thresholds_nominal(),
      lower_force_thresholds_acceleration(),
      upper_force_thresholds_acceleration(),
      lower_force_thresholds_nominal(),
      upper_force_thresholds_nominal()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      for( uint32_t i = 0; i < 7; i++){
      offset += serializeAvrFloat64(outbuffer + offset, this->lower_torque_thresholds_acceleration[i]);
      }
      for( uint32_t i = 0; i < 7; i++){
      offset += serializeAvrFloat64(outbuffer + offset, this->upper_torque_thresholds_acceleration[i]);
      }
      for( uint32_t i = 0; i < 7; i++){
      offset += serializeAvrFloat64(outbuffer + offset, this->lower_torque_thresholds_nominal[i]);
      }
      for( uint32_t i = 0; i < 7; i++){
      offset += serializeAvrFloat64(outbuffer + offset, this->upper_torque_thresholds_nominal[i]);
      }
      for( uint32_t i = 0; i < 6; i++){
      offset += serializeAvrFloat64(outbuffer + offset, this->lower_force_thresholds_acceleration[i]);
      }
      for( uint32_t i = 0; i < 6; i++){
      offset += serializeAvrFloat64(outbuffer + offset, this->upper_force_thresholds_acceleration[i]);
      }
      for( uint32_t i = 0; i < 6; i++){
      offset += serializeAvrFloat64(outbuffer + offset, this->lower_force_thresholds_nominal[i]);
      }
      for( uint32_t i = 0; i < 6; i++){
      offset += serializeAvrFloat64(outbuffer + offset, this->upper_force_thresholds_nominal[i]);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      for( uint32_t i = 0; i < 7; i++){
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->lower_torque_thresholds_acceleration[i]));
      }
      for( uint32_t i = 0; i < 7; i++){
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->upper_torque_thresholds_acceleration[i]));
      }
      for( uint32_t i = 0; i < 7; i++){
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->lower_torque_thresholds_nominal[i]));
      }
      for( uint32_t i = 0; i < 7; i++){
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->upper_torque_thresholds_nominal[i]));
      }
      for( uint32_t i = 0; i < 6; i++){
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->lower_force_thresholds_acceleration[i]));
      }
      for( uint32_t i = 0; i < 6; i++){
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->upper_force_thresholds_acceleration[i]));
      }
      for( uint32_t i = 0; i < 6; i++){
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->lower_force_thresholds_nominal[i]));
      }
      for( uint32_t i = 0; i < 6; i++){
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->upper_force_thresholds_nominal[i]));
      }
     return offset;
    }

    virtual const char * getType() override { return SETFULLCOLLISIONBEHAVIOR; };
    virtual const char * getMD5() override { return "bc875909c147220ba1720adcd48e47ea"; };

  };

  class SetFullCollisionBehaviorResponse : public ros::Msg
  {
    public:
      typedef bool _success_type;
      _success_type success;
      typedef const char* _error_type;
      _error_type error;

    SetFullCollisionBehaviorResponse():
      success(0),
      error("")
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
      uint32_t length_error = strlen(this->error);
      varToArr(outbuffer + offset, length_error);
      offset += 4;
      memcpy(outbuffer + offset, this->error, length_error);
      offset += length_error;
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
      uint32_t length_error;
      arrToVar(length_error, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_error; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_error-1]=0;
      this->error = (char *)(inbuffer + offset-1);
      offset += length_error;
     return offset;
    }

    virtual const char * getType() override { return SETFULLCOLLISIONBEHAVIOR; };
    virtual const char * getMD5() override { return "45872d25d65c97743cc71afc6d4e884d"; };

  };

  class SetFullCollisionBehavior {
    public:
    typedef SetFullCollisionBehaviorRequest Request;
    typedef SetFullCollisionBehaviorResponse Response;
  };

}
#endif
