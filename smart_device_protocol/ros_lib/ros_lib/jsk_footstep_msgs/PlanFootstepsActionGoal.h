#ifndef _ROS_jsk_footstep_msgs_PlanFootstepsActionGoal_h
#define _ROS_jsk_footstep_msgs_PlanFootstepsActionGoal_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "actionlib_msgs/GoalID.h"
#include "jsk_footstep_msgs/PlanFootstepsGoal.h"

namespace jsk_footstep_msgs
{

  class PlanFootstepsActionGoal : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef actionlib_msgs::GoalID _goal_id_type;
      _goal_id_type goal_id;
      typedef jsk_footstep_msgs::PlanFootstepsGoal _goal_type;
      _goal_type goal;

    PlanFootstepsActionGoal():
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

    virtual const char * getType() override { return "jsk_footstep_msgs/PlanFootstepsActionGoal"; };
    virtual const char * getMD5() override { return "4f600ecca20b1d57f2e38560d26042c4"; };

  };

}
#endif
