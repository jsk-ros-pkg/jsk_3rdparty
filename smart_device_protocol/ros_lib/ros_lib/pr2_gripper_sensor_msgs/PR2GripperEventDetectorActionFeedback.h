#ifndef _ROS_pr2_gripper_sensor_msgs_PR2GripperEventDetectorActionFeedback_h
#define _ROS_pr2_gripper_sensor_msgs_PR2GripperEventDetectorActionFeedback_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "actionlib_msgs/GoalStatus.h"
#include "pr2_gripper_sensor_msgs/PR2GripperEventDetectorFeedback.h"

namespace pr2_gripper_sensor_msgs
{

  class PR2GripperEventDetectorActionFeedback : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef actionlib_msgs::GoalStatus _status_type;
      _status_type status;
      typedef pr2_gripper_sensor_msgs::PR2GripperEventDetectorFeedback _feedback_type;
      _feedback_type feedback;

    PR2GripperEventDetectorActionFeedback():
      header(),
      status(),
      feedback()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      offset += this->status.serialize(outbuffer + offset);
      offset += this->feedback.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      offset += this->status.deserialize(inbuffer + offset);
      offset += this->feedback.deserialize(inbuffer + offset);
     return offset;
    }

    virtual const char * getType() override { return "pr2_gripper_sensor_msgs/PR2GripperEventDetectorActionFeedback"; };
    virtual const char * getMD5() override { return "b7cae5a018676f4d59b2f204229012d4"; };

  };

}
#endif
