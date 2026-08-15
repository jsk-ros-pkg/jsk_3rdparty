#ifndef _ROS_pr2_gripper_sensor_msgs_PR2GripperForceServoData_h
#define _ROS_pr2_gripper_sensor_msgs_PR2GripperForceServoData_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "ros/time.h"
#include "pr2_gripper_sensor_msgs/PR2GripperSensorRTState.h"

namespace pr2_gripper_sensor_msgs
{

  class PR2GripperForceServoData : public ros::Msg
  {
    public:
      typedef ros::Time _stamp_type;
      _stamp_type stamp;
      typedef float _left_fingertip_pad_force_type;
      _left_fingertip_pad_force_type left_fingertip_pad_force;
      typedef float _right_fingertip_pad_force_type;
      _right_fingertip_pad_force_type right_fingertip_pad_force;
      typedef float _joint_effort_type;
      _joint_effort_type joint_effort;
      typedef bool _force_achieved_type;
      _force_achieved_type force_achieved;
      typedef pr2_gripper_sensor_msgs::PR2GripperSensorRTState _rtstate_type;
      _rtstate_type rtstate;

    PR2GripperForceServoData():
      stamp(),
      left_fingertip_pad_force(0),
      right_fingertip_pad_force(0),
      joint_effort(0),
      force_achieved(0),
      rtstate()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->stamp.sec >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->stamp.sec >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->stamp.sec >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->stamp.sec >> (8 * 3)) & 0xFF;
      offset += sizeof(this->stamp.sec);
      *(outbuffer + offset + 0) = (this->stamp.nsec >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->stamp.nsec >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->stamp.nsec >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->stamp.nsec >> (8 * 3)) & 0xFF;
      offset += sizeof(this->stamp.nsec);
      offset += serializeAvrFloat64(outbuffer + offset, this->left_fingertip_pad_force);
      offset += serializeAvrFloat64(outbuffer + offset, this->right_fingertip_pad_force);
      offset += serializeAvrFloat64(outbuffer + offset, this->joint_effort);
      union {
        bool real;
        uint8_t base;
      } u_force_achieved;
      u_force_achieved.real = this->force_achieved;
      *(outbuffer + offset + 0) = (u_force_achieved.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->force_achieved);
      offset += this->rtstate.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      this->stamp.sec =  ((uint32_t) (*(inbuffer + offset)));
      this->stamp.sec |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->stamp.sec |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->stamp.sec |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->stamp.sec);
      this->stamp.nsec =  ((uint32_t) (*(inbuffer + offset)));
      this->stamp.nsec |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->stamp.nsec |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->stamp.nsec |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->stamp.nsec);
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->left_fingertip_pad_force));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->right_fingertip_pad_force));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->joint_effort));
      union {
        bool real;
        uint8_t base;
      } u_force_achieved;
      u_force_achieved.base = 0;
      u_force_achieved.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->force_achieved = u_force_achieved.real;
      offset += sizeof(this->force_achieved);
      offset += this->rtstate.deserialize(inbuffer + offset);
     return offset;
    }

    virtual const char * getType() override { return "pr2_gripper_sensor_msgs/PR2GripperForceServoData"; };
    virtual const char * getMD5() override { return "d3960eb2ecb6a9b4c27065619e47fd06"; };

  };

}
#endif
