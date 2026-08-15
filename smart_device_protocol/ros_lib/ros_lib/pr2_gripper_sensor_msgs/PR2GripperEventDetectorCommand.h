#ifndef _ROS_pr2_gripper_sensor_msgs_PR2GripperEventDetectorCommand_h
#define _ROS_pr2_gripper_sensor_msgs_PR2GripperEventDetectorCommand_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace pr2_gripper_sensor_msgs
{

  class PR2GripperEventDetectorCommand : public ros::Msg
  {
    public:
      typedef int8_t _trigger_conditions_type;
      _trigger_conditions_type trigger_conditions;
      typedef float _acceleration_trigger_magnitude_type;
      _acceleration_trigger_magnitude_type acceleration_trigger_magnitude;
      typedef float _slip_trigger_magnitude_type;
      _slip_trigger_magnitude_type slip_trigger_magnitude;
      enum { FINGER_SIDE_IMPACT_OR_ACC =  0 };
      enum { SLIP_AND_ACC =  1 };
      enum { FINGER_SIDE_IMPACT_OR_SLIP_OR_ACC =  2 };
      enum { SLIP =  3 };
      enum { ACC =  4 };

    PR2GripperEventDetectorCommand():
      trigger_conditions(0),
      acceleration_trigger_magnitude(0),
      slip_trigger_magnitude(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      union {
        int8_t real;
        uint8_t base;
      } u_trigger_conditions;
      u_trigger_conditions.real = this->trigger_conditions;
      *(outbuffer + offset + 0) = (u_trigger_conditions.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->trigger_conditions);
      offset += serializeAvrFloat64(outbuffer + offset, this->acceleration_trigger_magnitude);
      offset += serializeAvrFloat64(outbuffer + offset, this->slip_trigger_magnitude);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      union {
        int8_t real;
        uint8_t base;
      } u_trigger_conditions;
      u_trigger_conditions.base = 0;
      u_trigger_conditions.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->trigger_conditions = u_trigger_conditions.real;
      offset += sizeof(this->trigger_conditions);
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->acceleration_trigger_magnitude));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->slip_trigger_magnitude));
     return offset;
    }

    virtual const char * getType() override { return "pr2_gripper_sensor_msgs/PR2GripperEventDetectorCommand"; };
    virtual const char * getMD5() override { return "b91a7e1e863671a84c1d06e0cac3146e"; };

  };

}
#endif
