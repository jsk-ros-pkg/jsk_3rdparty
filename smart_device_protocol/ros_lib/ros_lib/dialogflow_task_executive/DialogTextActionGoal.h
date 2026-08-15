#ifndef _ROS_dialogflow_task_executive_DialogTextActionGoal_h
#define _ROS_dialogflow_task_executive_DialogTextActionGoal_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "actionlib_msgs/GoalID.h"
#include "dialogflow_task_executive/DialogTextGoal.h"

namespace dialogflow_task_executive
{

  class DialogTextActionGoal : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef actionlib_msgs::GoalID _goal_id_type;
      _goal_id_type goal_id;
      typedef dialogflow_task_executive::DialogTextGoal _goal_type;
      _goal_type goal;

    DialogTextActionGoal():
      header(),
      goal_id(),
      goal()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      offset += this->goal_id.serialize(outbuffer + offset);
      offset += this->goal.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      offset += this->goal_id.deserialize(inbuffer + offset);
      offset += this->goal.deserialize(inbuffer + offset);
     return offset;
    }

    virtual const char * getType() override { return "dialogflow_task_executive/DialogTextActionGoal"; };
    virtual const char * getMD5() override { return "21f9decb4d62a44aa93e2bd8088c7df9"; };

  };

}
#endif
