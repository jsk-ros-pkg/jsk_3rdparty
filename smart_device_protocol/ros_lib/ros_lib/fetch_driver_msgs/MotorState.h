#ifndef _ROS_fetch_driver_msgs_MotorState_h
#define _ROS_fetch_driver_msgs_MotorState_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace fetch_driver_msgs
{

  class MotorState : public ros::Msg
  {
    public:
      typedef const char* _name_type;
      _name_type name;
      typedef uint8_t _state_type;
      _state_type state;
      typedef uint8_t _error_type;
      _error_type error;
      typedef float _velocity_type;
      _velocity_type velocity;
      typedef float _effort_type;
      _effort_type effort;
      typedef float _position_type;
      _position_type position;
      typedef float _temperature_type;
      _temperature_type temperature;
      typedef float _motor_ratio_type;
      _motor_ratio_type motor_ratio;
      typedef float _motor_angle_offset_type;
      _motor_angle_offset_type motor_angle_offset;
      typedef float _motor_angle_offset_estimated_type;
      _motor_angle_offset_estimated_type motor_angle_offset_estimated;
      typedef float _joint_ratio_type;
      _joint_ratio_type joint_ratio;

    MotorState():
      name(""),
      state(0),
      error(0),
      velocity(0),
      effort(0),
      position(0),
      temperature(0),
      motor_ratio(0),
      motor_angle_offset(0),
      motor_angle_offset_estimated(0),
      joint_ratio(0)
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
      *(outbuffer + offset + 0) = (this->error >> (8 * 0)) & 0xFF;
      offset += sizeof(this->error);
      union {
        float real;
        uint32_t base;
      } u_velocity;
      u_velocity.real = this->velocity;
      *(outbuffer + offset + 0) = (u_velocity.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_velocity.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_velocity.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_velocity.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->velocity);
      union {
        float real;
        uint32_t base;
      } u_effort;
      u_effort.real = this->effort;
      *(outbuffer + offset + 0) = (u_effort.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_effort.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_effort.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_effort.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->effort);
      union {
        float real;
        uint32_t base;
      } u_position;
      u_position.real = this->position;
      *(outbuffer + offset + 0) = (u_position.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_position.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_position.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_position.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->position);
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
      } u_motor_ratio;
      u_motor_ratio.real = this->motor_ratio;
      *(outbuffer + offset + 0) = (u_motor_ratio.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_motor_ratio.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_motor_ratio.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_motor_ratio.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->motor_ratio);
      union {
        float real;
        uint32_t base;
      } u_motor_angle_offset;
      u_motor_angle_offset.real = this->motor_angle_offset;
      *(outbuffer + offset + 0) = (u_motor_angle_offset.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_motor_angle_offset.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_motor_angle_offset.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_motor_angle_offset.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->motor_angle_offset);
      union {
        float real;
        uint32_t base;
      } u_motor_angle_offset_estimated;
      u_motor_angle_offset_estimated.real = this->motor_angle_offset_estimated;
      *(outbuffer + offset + 0) = (u_motor_angle_offset_estimated.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_motor_angle_offset_estimated.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_motor_angle_offset_estimated.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_motor_angle_offset_estimated.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->motor_angle_offset_estimated);
      union {
        float real;
        uint32_t base;
      } u_joint_ratio;
      u_joint_ratio.real = this->joint_ratio;
      *(outbuffer + offset + 0) = (u_joint_ratio.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_joint_ratio.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_joint_ratio.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_joint_ratio.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->joint_ratio);
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
      this->error =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->error);
      union {
        float real;
        uint32_t base;
      } u_velocity;
      u_velocity.base = 0;
      u_velocity.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_velocity.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_velocity.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_velocity.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->velocity = u_velocity.real;
      offset += sizeof(this->velocity);
      union {
        float real;
        uint32_t base;
      } u_effort;
      u_effort.base = 0;
      u_effort.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_effort.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_effort.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_effort.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->effort = u_effort.real;
      offset += sizeof(this->effort);
      union {
        float real;
        uint32_t base;
      } u_position;
      u_position.base = 0;
      u_position.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_position.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_position.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_position.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->position = u_position.real;
      offset += sizeof(this->position);
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
      } u_motor_ratio;
      u_motor_ratio.base = 0;
      u_motor_ratio.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_motor_ratio.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_motor_ratio.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_motor_ratio.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->motor_ratio = u_motor_ratio.real;
      offset += sizeof(this->motor_ratio);
      union {
        float real;
        uint32_t base;
      } u_motor_angle_offset;
      u_motor_angle_offset.base = 0;
      u_motor_angle_offset.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_motor_angle_offset.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_motor_angle_offset.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_motor_angle_offset.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->motor_angle_offset = u_motor_angle_offset.real;
      offset += sizeof(this->motor_angle_offset);
      union {
        float real;
        uint32_t base;
      } u_motor_angle_offset_estimated;
      u_motor_angle_offset_estimated.base = 0;
      u_motor_angle_offset_estimated.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_motor_angle_offset_estimated.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_motor_angle_offset_estimated.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_motor_angle_offset_estimated.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->motor_angle_offset_estimated = u_motor_angle_offset_estimated.real;
      offset += sizeof(this->motor_angle_offset_estimated);
      union {
        float real;
        uint32_t base;
      } u_joint_ratio;
      u_joint_ratio.base = 0;
      u_joint_ratio.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_joint_ratio.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_joint_ratio.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_joint_ratio.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->joint_ratio = u_joint_ratio.real;
      offset += sizeof(this->joint_ratio);
     return offset;
    }

    virtual const char * getType() override { return "fetch_driver_msgs/MotorState"; };
    virtual const char * getMD5() override { return "b726a0cde2dd21efcdc6db33af6de8d2"; };

  };

}
#endif
