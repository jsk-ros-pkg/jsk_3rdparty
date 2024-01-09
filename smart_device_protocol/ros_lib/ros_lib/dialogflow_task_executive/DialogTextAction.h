#ifndef _ROS_dialogflow_task_executive_DialogTextAction_h
#define _ROS_dialogflow_task_executive_DialogTextAction_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "dialogflow_task_executive/DialogTextActionGoal.h"
#include "dialogflow_task_executive/DialogTextActionResult.h"
#include "dialogflow_task_executive/DialogTextActionFeedback.h"

namespace dialogflow_task_executive
{

  class DialogTextAction : public ros::Msg
  {
    public:
      typedef dialogflow_task_executive::DialogTextActionGoal _action_goal_type;
      _action_goal_type action_goal;
      typedef dialogflow_task_executive::DialogTextActionResult _action_result_type;
      _action_result_type action_result;
      typedef dialogflow_task_executive::DialogTextActionFeedback _action_feedback_type;
      _action_feedback_type action_feedback;

    DialogTextAction():
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

    virtual const char * getType() override { return "dialogflow_task_executive/DialogTextAction"; };
    virtual const char * getMD5() override { return "6e457c92f187469e064600d4d7bad518"; };

  };

}
#endif
