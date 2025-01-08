#ifndef _ROS_switchbot_ros_SwitchBotCommandActionGoal_h
#define _ROS_switchbot_ros_SwitchBotCommandActionGoal_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "actionlib_msgs/GoalID.h"
#include "switchbot_ros/SwitchBotCommandGoal.h"

namespace switchbot_ros
{

  class SwitchBotCommandActionGoal : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef actionlib_msgs::GoalID _goal_id_type;
      _goal_id_type goal_id;
      typedef switchbot_ros::SwitchBotCommandGoal _goal_type;
      _goal_type goal;

    SwitchBotCommandActionGoal():
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

    virtual const char * getType() override { return "switchbot_ros/SwitchBotCommandActionGoal"; };
    virtual const char * getMD5() override { return "4a48aee4fa1fd1daf63f93b6466220d3"; };

  };

}
#endif
