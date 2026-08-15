#ifndef _ROS_robot_calibration_msgs_GripperLedCommandAction_h
#define _ROS_robot_calibration_msgs_GripperLedCommandAction_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "robot_calibration_msgs/GripperLedCommandActionGoal.h"
#include "robot_calibration_msgs/GripperLedCommandActionResult.h"
#include "robot_calibration_msgs/GripperLedCommandActionFeedback.h"

namespace robot_calibration_msgs
{

  class GripperLedCommandAction : public ros::Msg
  {
    public:
      typedef robot_calibration_msgs::GripperLedCommandActionGoal _action_goal_type;
      _action_goal_type action_goal;
      typedef robot_calibration_msgs::GripperLedCommandActionResult _action_result_type;
      _action_result_type action_result;
      typedef robot_calibration_msgs::GripperLedCommandActionFeedback _action_feedback_type;
      _action_feedback_type action_feedback;

    GripperLedCommandAction():
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

    virtual const char * getType() override { return "robot_calibration_msgs/GripperLedCommandAction"; };
    virtual const char * getMD5() override { return "48f166d6c125d28b70639966a1197497"; };

  };

}
#endif
