#ifndef _ROS_ethercat_hardware_ActuatorInfo_h
#define _ROS_ethercat_hardware_ActuatorInfo_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace ethercat_hardware
{

  class ActuatorInfo : public ros::Msg
  {
    public:
      typedef uint32_t _id_type;
      _id_type id;
      typedef const char* _name_type;
      _name_type name;
      typedef const char* _robot_name_type;
      _robot_name_type robot_name;
      typedef const char* _motor_make_type;
      _motor_make_type motor_make;
      typedef const char* _motor_model_type;
      _motor_model_type motor_model;
      typedef float _max_current_type;
      _max_current_type max_current;
      typedef float _speed_constant_type;
      _speed_constant_type speed_constant;
      typedef float _motor_resistance_type;
      _motor_resistance_type motor_resistance;
      typedef float _motor_torque_constant_type;
      _motor_torque_constant_type motor_torque_constant;
      typedef float _encoder_reduction_type;
      _encoder_reduction_type encoder_reduction;
      typedef float _pulses_per_revolution_type;
      _pulses_per_revolution_type pulses_per_revolution;

    ActuatorInfo():
      id(0),
      name(""),
      robot_name(""),
      motor_make(""),
      motor_model(""),
      max_current(0),
      speed_constant(0),
      motor_resistance(0),
      motor_torque_constant(0),
      encoder_reduction(0),
      pulses_per_revolution(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->id >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->id >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->id >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->id >> (8 * 3)) & 0xFF;
      offset += sizeof(this->id);
      uint32_t length_name = strlen(this->name);
      varToArr(outbuffer + offset, length_name);
      offset += 4;
      memcpy(outbuffer + offset, this->name, length_name);
      offset += length_name;
      uint32_t length_robot_name = strlen(this->robot_name);
      varToArr(outbuffer + offset, length_robot_name);
      offset += 4;
      memcpy(outbuffer + offset, this->robot_name, length_robot_name);
      offset += length_robot_name;
      uint32_t length_motor_make = strlen(this->motor_make);
      varToArr(outbuffer + offset, length_motor_make);
      offset += 4;
      memcpy(outbuffer + offset, this->motor_make, length_motor_make);
      offset += length_motor_make;
      uint32_t length_motor_model = strlen(this->motor_model);
      varToArr(outbuffer + offset, length_motor_model);
      offset += 4;
      memcpy(outbuffer + offset, this->motor_model, length_motor_model);
      offset += length_motor_model;
      offset += serializeAvrFloat64(outbuffer + offset, this->max_current);
      offset += serializeAvrFloat64(outbuffer + offset, this->speed_constant);
      offset += serializeAvrFloat64(outbuffer + offset, this->motor_resistance);
      offset += serializeAvrFloat64(outbuffer + offset, this->motor_torque_constant);
      offset += serializeAvrFloat64(outbuffer + offset, this->encoder_reduction);
      offset += serializeAvrFloat64(outbuffer + offset, this->pulses_per_revolution);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      this->id =  ((uint32_t) (*(inbuffer + offset)));
      this->id |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->id |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->id |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->id);
      uint32_t length_name;
      arrToVar(length_name, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_name; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_name-1]=0;
      this->name = (char *)(inbuffer + offset-1);
      offset += length_name;
      uint32_t length_robot_name;
      arrToVar(length_robot_name, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_robot_name; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_robot_name-1]=0;
      this->robot_name = (char *)(inbuffer + offset-1);
      offset += length_robot_name;
      uint32_t length_motor_make;
      arrToVar(length_motor_make, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_motor_make; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_motor_make-1]=0;
      this->motor_make = (char *)(inbuffer + offset-1);
      offset += length_motor_make;
      uint32_t length_motor_model;
      arrToVar(length_motor_model, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_motor_model; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_motor_model-1]=0;
      this->motor_model = (char *)(inbuffer + offset-1);
      offset += length_motor_model;
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->max_current));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->speed_constant));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->motor_resistance));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->motor_torque_constant));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->encoder_reduction));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->pulses_per_revolution));
     return offset;
    }

    virtual const char * getType() override { return "ethercat_hardware/ActuatorInfo"; };
    virtual const char * getMD5() override { return "40f44d8ec4380adc0b63713486eecb09"; };

  };

}
#endif
