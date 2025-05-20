#ifndef _ROS_pr2_gripper_sensor_msgs_PR2GripperFindContactAction_h
#define _ROS_pr2_gripper_sensor_msgs_PR2GripperFindContactAction_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "pr2_gripper_sensor_msgs/PR2GripperFindContactActionGoal.h"
#include "pr2_gripper_sensor_msgs/PR2GripperFindContactActionResult.h"
#include "pr2_gripper_sensor_msgs/PR2GripperFindContactActionFeedback.h"

namespace pr2_gripper_sensor_msgs
{

  class PR2GripperFindContactAction : public ros::Msg
  {
    public:
      typedef pr2_gripper_sensor_msgs::PR2GripperFindContactActionGoal _action_goal_type;
      _action_goal_type action_goal;
      typedef pr2_gripper_sensor_msgs::PR2GripperFindContactActionResult _action_result_type;
      _action_result_type action_result;
      typedef pr2_gripper_sensor_msgs::PR2GripperFindContactActionFeedback _action_feedback_type;
      _action_feedback_type action_feedback;

    PR2GripperFindContactAction():
      action_goal(),
      action_result(),
      action_feedback()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->action_goal.serialize(outbuffer + offset);
      offset += this->action_result.serialize(outbuffer + offset);
      offset += this->action_feedback.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += this->action_goal.deserialize(inbuffer + offset);
      offset += this->action_result.deserialize(inbuffer + offset);
      offset += this->action_feedback.deserialize(inbuffer + offset);
     return offset;
    }

    virtual const char * getType() override { return "pr2_gripper_sensor_msgs/PR2GripperFindContactAction"; };
    virtual const char * getMD5() override { return "99ab2df1bbde46c447b38f28b7896d16"; };

  };

}
#endif
