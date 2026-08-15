#ifndef _ROS_pr2_gripper_sensor_msgs_PR2GripperEventDetectorActionResult_h
#define _ROS_pr2_gripper_sensor_msgs_PR2GripperEventDetectorActionResult_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "actionlib_msgs/GoalStatus.h"
#include "pr2_gripper_sensor_msgs/PR2GripperEventDetectorResult.h"

namespace pr2_gripper_sensor_msgs
{

  class PR2GripperEventDetectorActionResult : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef actionlib_msgs::GoalStatus _status_type;
      _status_type status;
      typedef pr2_gripper_sensor_msgs::PR2GripperEventDetectorResult _result_type;
      _result_type result;

    PR2GripperEventDetectorActionResult():
      header(),
      status(),
      result()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      offset += this->status.serialize(outbuffer + offset);
      offset += this->result.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      offset += this->status.deserialize(inbuffer + offset);
      offset += this->result.deserialize(inbuffer + offset);
     return offset;
    }

    virtual const char * getType() override { return "pr2_gripper_sensor_msgs/PR2GripperEventDetectorActionResult"; };
    virtual const char * getMD5() override { return "a8c6f42e274ecd861ad072720ef9894b"; };

  };

}
#endif
