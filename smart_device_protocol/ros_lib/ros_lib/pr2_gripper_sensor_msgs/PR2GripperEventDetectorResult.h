#ifndef _ROS_pr2_gripper_sensor_msgs_PR2GripperEventDetectorResult_h
#define _ROS_pr2_gripper_sensor_msgs_PR2GripperEventDetectorResult_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "pr2_gripper_sensor_msgs/PR2GripperEventDetectorData.h"

namespace pr2_gripper_sensor_msgs
{

  class PR2GripperEventDetectorResult : public ros::Msg
  {
    public:
      typedef pr2_gripper_sensor_msgs::PR2GripperEventDetectorData _data_type;
      _data_type data;

    PR2GripperEventDetectorResult():
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

    virtual const char * getType() override { return "pr2_gripper_sensor_msgs/PR2GripperEventDetectorResult"; };
    virtual const char * getMD5() override { return "817b45a51c75a067eb5dfb8e18b14aa1"; };

  };

}
#endif
