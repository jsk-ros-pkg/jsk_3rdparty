#ifndef _ROS_pr2_gripper_sensor_msgs_PR2GripperFindContactFeedback_h
#define _ROS_pr2_gripper_sensor_msgs_PR2GripperFindContactFeedback_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "pr2_gripper_sensor_msgs/PR2GripperFindContactData.h"

namespace pr2_gripper_sensor_msgs
{

  class PR2GripperFindContactFeedback : public ros::Msg
  {
    public:
      typedef pr2_gripper_sensor_msgs::PR2GripperFindContactData _data_type;
      _data_type data;

    PR2GripperFindContactFeedback():
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

    virtual const char * getType() override { return "pr2_gripper_sensor_msgs/PR2GripperFindContactFeedback"; };
    virtual const char * getMD5() override { return "a1cc8c2fc9268b550e6167f268f97574"; };

  };

}
#endif
