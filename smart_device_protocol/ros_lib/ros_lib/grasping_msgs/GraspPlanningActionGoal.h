#ifndef _ROS_grasping_msgs_GraspPlanningActionGoal_h
#define _ROS_grasping_msgs_GraspPlanningActionGoal_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "actionlib_msgs/GoalID.h"
#include "grasping_msgs/GraspPlanningGoal.h"

namespace grasping_msgs
{

  class GraspPlanningActionGoal : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef actionlib_msgs::GoalID _goal_id_type;
      _goal_id_type goal_id;
      typedef grasping_msgs::GraspPlanningGoal _goal_type;
      _goal_type goal;

    GraspPlanningActionGoal():
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

    virtual const char * getType() override { return "grasping_msgs/GraspPlanningActionGoal"; };
    virtual const char * getMD5() override { return "caad3d76fe6a67352a76302e837cccbe"; };

  };

}
#endif
