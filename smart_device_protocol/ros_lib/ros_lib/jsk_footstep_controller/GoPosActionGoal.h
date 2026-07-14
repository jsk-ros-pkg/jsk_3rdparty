#ifndef _ROS_jsk_footstep_controller_GoPosActionGoal_h
#define _ROS_jsk_footstep_controller_GoPosActionGoal_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "actionlib_msgs/GoalID.h"
#include "jsk_footstep_controller/GoPosGoal.h"

namespace jsk_footstep_controller
{

  class GoPosActionGoal : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef actionlib_msgs::GoalID _goal_id_type;
      _goal_id_type goal_id;
      typedef jsk_footstep_controller::GoPosGoal _goal_type;
      _goal_type goal;

    GoPosActionGoal():
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

    virtual const char * getType() override { return "jsk_footstep_controller/GoPosActionGoal"; };
    virtual const char * getMD5() override { return "1e10617102395e85ccbe04b2666246b4"; };

  };

}
#endif
