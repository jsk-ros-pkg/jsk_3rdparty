#ifndef _ROS_pr2_controllers_msgs_JointControllerState_h
#define _ROS_pr2_controllers_msgs_JointControllerState_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"

namespace pr2_controllers_msgs
{

  class JointControllerState : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef float _set_point_type;
      _set_point_type set_point;
      typedef float _process_value_type;
      _process_value_type process_value;
      typedef float _process_value_dot_type;
      _process_value_dot_type process_value_dot;
      typedef float _error_type;
      _error_type error;
      typedef float _time_step_type;
      _time_step_type time_step;
      typedef float _command_type;
      _command_type command;
      typedef float _p_type;
      _p_type p;
      typedef float _i_type;
      _i_type i;
      typedef float _d_type;
      _d_type d;
      typedef float _i_clamp_type;
      _i_clamp_type i_clamp;

    JointControllerState():
      header(),
      set_point(0),
      process_value(0),
      process_value_dot(0),
      error(0),
      time_step(0),
      command(0),
      p(0),
      i(0),
      d(0),
      i_clamp(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      offset += serializeAvrFloat64(outbuffer + offset, this->set_point);
      offset += serializeAvrFloat64(outbuffer + offset, this->process_value);
      offset += serializeAvrFloat64(outbuffer + offset, this->process_value_dot);
      offset += serializeAvrFloat64(outbuffer + offset, this->error);
      offset += serializeAvrFloat64(outbuffer + offset, this->time_step);
      offset += serializeAvrFloat64(outbuffer + offset, this->command);
      offset += serializeAvrFloat64(outbuffer + offset, this->p);
      offset += serializeAvrFloat64(outbuffer + offset, this->i);
      offset += serializeAvrFloat64(outbuffer + offset, this->d);
      offset += serializeAvrFloat64(outbuffer + offset, this->i_clamp);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->set_point));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->process_value));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->process_value_dot));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->error));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->time_step));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->command));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->p));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->i));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->d));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->i_clamp));
     return offset;
    }

    virtual const char * getType() override { return "pr2_controllers_msgs/JointControllerState"; };
    virtual const char * getMD5() override { return "c0d034a7bf20aeb1c37f3eccb7992b69"; };

  };

}
#endif
