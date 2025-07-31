#ifndef _ROS_pr2_gripper_sensor_msgs_PR2GripperForceServoActionGoal_h
#define _ROS_pr2_gripper_sensor_msgs_PR2GripperForceServoActionGoal_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "actionlib_msgs/GoalID.h"
#include "pr2_gripper_sensor_msgs/PR2GripperForceServoGoal.h"

namespace pr2_gripper_sensor_msgs
{

  class PR2GripperForceServoActionGoal : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef actionlib_msgs::GoalID _goal_id_type;
      _goal_id_type goal_id;
      typedef pr2_gripper_sensor_msgs::PR2GripperForceServoGoal _goal_type;
      _goal_type goal;

    PR2GripperForceServoActionGoal():
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

    virtual const char * getType() override { return "pr2_gripper_sensor_msgs/PR2GripperForceServoActionGoal"; };
    virtual const char * getMD5() override { return "6fef46f6d468335bf123ba62175e1dc9"; };

  };

}
#endif
