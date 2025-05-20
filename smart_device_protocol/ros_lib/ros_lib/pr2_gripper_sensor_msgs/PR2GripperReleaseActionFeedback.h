#ifndef _ROS_pr2_gripper_sensor_msgs_PR2GripperReleaseActionFeedback_h
#define _ROS_pr2_gripper_sensor_msgs_PR2GripperReleaseActionFeedback_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "actionlib_msgs/GoalStatus.h"
#include "pr2_gripper_sensor_msgs/PR2GripperReleaseFeedback.h"

namespace pr2_gripper_sensor_msgs
{

  class PR2GripperReleaseActionFeedback : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef actionlib_msgs::GoalStatus _status_type;
      _status_type status;
      typedef pr2_gripper_sensor_msgs::PR2GripperReleaseFeedback _feedback_type;
      _feedback_type feedback;

    PR2GripperReleaseActionFeedback():
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

    virtual const char * getType() override { return "pr2_gripper_sensor_msgs/PR2GripperReleaseActionFeedback"; };
    virtual const char * getMD5() override { return "83231ee429239525d42b1713d6791069"; };

  };

}
#endif
