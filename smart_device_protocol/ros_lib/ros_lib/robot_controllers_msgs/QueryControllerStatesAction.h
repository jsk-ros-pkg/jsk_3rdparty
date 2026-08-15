#ifndef _ROS_robot_controllers_msgs_QueryControllerStatesAction_h
#define _ROS_robot_controllers_msgs_QueryControllerStatesAction_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "robot_controllers_msgs/QueryControllerStatesActionGoal.h"
#include "robot_controllers_msgs/QueryControllerStatesActionResult.h"
#include "robot_controllers_msgs/QueryControllerStatesActionFeedback.h"

namespace robot_controllers_msgs
{

  class QueryControllerStatesAction : public ros::Msg
  {
    public:
      typedef robot_controllers_msgs::QueryControllerStatesActionGoal _action_goal_type;
      _action_goal_type action_goal;
      typedef robot_controllers_msgs::QueryControllerStatesActionResult _action_result_type;
      _action_result_type action_result;
      typedef robot_controllers_msgs::QueryControllerStatesActionFeedback _action_feedback_type;
      _action_feedback_type action_feedback;

    QueryControllerStatesAction():
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

    virtual const char * getType() override { return "robot_controllers_msgs/QueryControllerStatesAction"; };
    virtual const char * getMD5() override { return "09c0acaa5ed1f30d01515baab9a95895"; };

  };

}
#endif
