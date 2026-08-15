#ifndef _ROS_pr2_gripper_sensor_msgs_PR2GripperGrabData_h
#define _ROS_pr2_gripper_sensor_msgs_PR2GripperGrabData_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "pr2_gripper_sensor_msgs/PR2GripperSensorRTState.h"

namespace pr2_gripper_sensor_msgs
{

  class PR2GripperGrabData : public ros::Msg
  {
    public:
      typedef pr2_gripper_sensor_msgs::PR2GripperSensorRTState _rtstate_type;
      _rtstate_type rtstate;

    PR2GripperGrabData():
      rtstate()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->rtstate.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += this->rtstate.deserialize(inbuffer + offset);
     return offset;
    }

    virtual const char * getType() override { return "pr2_gripper_sensor_msgs/PR2GripperGrabData"; };
    virtual const char * getMD5() override { return "2c917fd7a48bc8daa7ae36787c8b7a82"; };

  };

}
#endif
