#ifndef _ROS_robot_calibration_msgs_GripperLedCommandActionGoal_h
#define _ROS_robot_calibration_msgs_GripperLedCommandActionGoal_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "actionlib_msgs/GoalID.h"
#include "robot_calibration_msgs/GripperLedCommandGoal.h"

namespace robot_calibration_msgs
{

  class GripperLedCommandActionGoal : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef actionlib_msgs::GoalID _goal_id_type;
      _goal_id_type goal_id;
      typedef robot_calibration_msgs::GripperLedCommandGoal _goal_type;
      _goal_type goal;

    GripperLedCommandActionGoal():
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

    virtual const char * getType() override { return "robot_calibration_msgs/GripperLedCommandActionGoal"; };
    virtual const char * getMD5() override { return "a6f43a5345c808a3839fc4ee405e8697"; };

  };

}
#endif
