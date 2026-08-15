#ifndef _ROS_robot_controllers_msgs_QueryControllerStatesActionGoal_h
#define _ROS_robot_controllers_msgs_QueryControllerStatesActionGoal_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "actionlib_msgs/GoalID.h"
#include "robot_controllers_msgs/QueryControllerStatesGoal.h"

namespace robot_controllers_msgs
{

  class QueryControllerStatesActionGoal : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef actionlib_msgs::GoalID _goal_id_type;
      _goal_id_type goal_id;
      typedef robot_controllers_msgs::QueryControllerStatesGoal _goal_type;
      _goal_type goal;

    QueryControllerStatesActionGoal():
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

    virtual const char * getType() override { return "robot_controllers_msgs/QueryControllerStatesActionGoal"; };
    virtual const char * getMD5() override { return "291a917d724f37ef2a137fb40fae4e4a"; };

  };

}
#endif
