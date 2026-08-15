#ifndef _ROS_robot_controllers_msgs_DiffDriveLimiterParams_h
#define _ROS_robot_controllers_msgs_DiffDriveLimiterParams_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace robot_controllers_msgs
{

  class DiffDriveLimiterParams : public ros::Msg
  {
    public:
      typedef float _max_linear_velocity_type;
      _max_linear_velocity_type max_linear_velocity;
      typedef float _max_linear_acceleration_type;
      _max_linear_acceleration_type max_linear_acceleration;
      typedef float _max_angular_velocity_type;
      _max_angular_velocity_type max_angular_velocity;
      typedef float _max_angular_acceleration_type;
      _max_angular_acceleration_type max_angular_acceleration;
      typedef float _max_wheel_velocity_type;
      _max_wheel_velocity_type max_wheel_velocity;
      typedef float _track_width_type;
      _track_width_type track_width;
      typedef bool _angular_velocity_limits_linear_velocity_type;
      _angular_velocity_limits_linear_velocity_type angular_velocity_limits_linear_velocity;
      typedef bool _scale_to_wheel_velocity_limits_type;
      _scale_to_wheel_velocity_limits_type scale_to_wheel_velocity_limits;

    DiffDriveLimiterParams():
      max_linear_velocity(0),
      max_linear_acceleration(0),
      max_angular_velocity(0),
      max_angular_acceleration(0),
      max_wheel_velocity(0),
      track_width(0),
      angular_velocity_limits_linear_velocity(0),
      scale_to_wheel_velocity_limits(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += serializeAvrFloat64(outbuffer + offset, this->max_linear_velocity);
      offset += serializeAvrFloat64(outbuffer + offset, this->max_linear_acceleration);
      offset += serializeAvrFloat64(outbuffer + offset, this->max_angular_velocity);
      offset += serializeAvrFloat64(outbuffer + offset, this->max_angular_acceleration);
      offset += serializeAvrFloat64(outbuffer + offset, this->max_wheel_velocity);
      offset += serializeAvrFloat64(outbuffer + offset, this->track_width);
      union {
        bool real;
        uint8_t base;
      } u_angular_velocity_limits_linear_velocity;
      u_angular_velocity_limits_linear_velocity.real = this->angular_velocity_limits_linear_velocity;
      *(outbuffer + offset + 0) = (u_angular_velocity_limits_linear_velocity.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->angular_velocity_limits_linear_velocity);
      union {
        bool real;
        uint8_t base;
      } u_scale_to_wheel_velocity_limits;
      u_scale_to_wheel_velocity_limits.real = this->scale_to_wheel_velocity_limits;
      *(outbuffer + offset + 0) = (u_scale_to_wheel_velocity_limits.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->scale_to_wheel_velocity_limits);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->max_linear_velocity));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->max_linear_acceleration));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->max_angular_velocity));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->max_angular_acceleration));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->max_wheel_velocity));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->track_width));
      union {
        bool real;
        uint8_t base;
      } u_angular_velocity_limits_linear_velocity;
      u_angular_velocity_limits_linear_velocity.base = 0;
      u_angular_velocity_limits_linear_velocity.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->angular_velocity_limits_linear_velocity = u_angular_velocity_limits_linear_velocity.real;
      offset += sizeof(this->angular_velocity_limits_linear_velocity);
      union {
        bool real;
        uint8_t base;
      } u_scale_to_wheel_velocity_limits;
      u_scale_to_wheel_velocity_limits.base = 0;
      u_scale_to_wheel_velocity_limits.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->scale_to_wheel_velocity_limits = u_scale_to_wheel_velocity_limits.real;
      offset += sizeof(this->scale_to_wheel_velocity_limits);
     return offset;
    }

    virtual const char * getType() override { return "robot_controllers_msgs/DiffDriveLimiterParams"; };
    virtual const char * getMD5() override { return "c438ebbdf2d3d45fdfb67f5ba9e6ca3d"; };

  };

}
#endif
