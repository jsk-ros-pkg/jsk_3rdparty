#ifndef _ROS_pr2_gripper_sensor_msgs_PR2GripperGrabAction_h
#define _ROS_pr2_gripper_sensor_msgs_PR2GripperGrabAction_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "pr2_gripper_sensor_msgs/PR2GripperGrabActionGoal.h"
#include "pr2_gripper_sensor_msgs/PR2GripperGrabActionResult.h"
#include "pr2_gripper_sensor_msgs/PR2GripperGrabActionFeedback.h"

namespace pr2_gripper_sensor_msgs
{

  class PR2GripperGrabAction : public ros::Msg
  {
    public:
      typedef pr2_gripper_sensor_msgs::PR2GripperGrabActionGoal _action_goal_type;
      _action_goal_type action_goal;
      typedef pr2_gripper_sensor_msgs::PR2GripperGrabActionResult _action_result_type;
      _action_result_type action_result;
      typedef pr2_gripper_sensor_msgs::PR2GripperGrabActionFeedback _action_feedback_type;
      _action_feedback_type action_feedback;

    PR2GripperGrabAction():
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

    virtual const char * getType() override { return "pr2_gripper_sensor_msgs/PR2GripperGrabAction"; };
    virtual const char * getMD5() override { return "f467562414aabe5b90666be976b0c379"; };

  };

}
#endif
