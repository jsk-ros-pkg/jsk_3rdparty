#ifndef _ROS_pr2_gripper_sensor_msgs_PR2GripperSlipServoFeedback_h
#define _ROS_pr2_gripper_sensor_msgs_PR2GripperSlipServoFeedback_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "pr2_gripper_sensor_msgs/PR2GripperSlipServoData.h"

namespace pr2_gripper_sensor_msgs
{

  class PR2GripperSlipServoFeedback : public ros::Msg
  {
    public:
      typedef pr2_gripper_sensor_msgs::PR2GripperSlipServoData _data_type;
      _data_type data;

    PR2GripperSlipServoFeedback():
      data()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->data.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += this->data.deserialize(inbuffer + offset);
     return offset;
    }

    virtual const char * getType() override { return "pr2_gripper_sensor_msgs/PR2GripperSlipServoFeedback"; };
    virtual const char * getMD5() override { return "1b10af616c7e94f609790b12cde04c6d"; };

  };

}
#endif
