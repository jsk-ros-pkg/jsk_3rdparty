#ifndef _ROS_pr2_gripper_sensor_msgs_PR2GripperSensorRTState_h
#define _ROS_pr2_gripper_sensor_msgs_PR2GripperSensorRTState_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace pr2_gripper_sensor_msgs
{

  class PR2GripperSensorRTState : public ros::Msg
  {
    public:
      typedef int8_t _realtime_controller_state_type;
      _realtime_controller_state_type realtime_controller_state;
      enum { DISABLED =  0 };
      enum { POSITION_SERVO =  3 };
      enum { FORCE_SERVO =  4 };
      enum { FIND_CONTACT =  5 };
      enum { SLIP_SERVO =  6 };

    PR2GripperSensorRTState():
      realtime_controller_state(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      union {
        int8_t real;
        uint8_t base;
      } u_realtime_controller_state;
      u_realtime_controller_state.real = this->realtime_controller_state;
      *(outbuffer + offset + 0) = (u_realtime_controller_state.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->realtime_controller_state);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      union {
        int8_t real;
        uint8_t base;
      } u_realtime_controller_state;
      u_realtime_controller_state.base = 0;
      u_realtime_controller_state.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->realtime_controller_state = u_realtime_controller_state.real;
      offset += sizeof(this->realtime_controller_state);
     return offset;
    }

    virtual const char * getType() override { return "pr2_gripper_sensor_msgs/PR2GripperSensorRTState"; };
    virtual const char * getMD5() override { return "8109436c1f7237c52c00d885ed5755d7"; };

  };

}
#endif
