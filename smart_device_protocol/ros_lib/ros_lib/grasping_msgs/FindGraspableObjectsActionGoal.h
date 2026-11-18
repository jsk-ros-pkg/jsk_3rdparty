#ifndef _ROS_grasping_msgs_FindGraspableObjectsActionGoal_h
#define _ROS_grasping_msgs_FindGraspableObjectsActionGoal_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "actionlib_msgs/GoalID.h"
#include "grasping_msgs/FindGraspableObjectsGoal.h"

namespace grasping_msgs
{

  class FindGraspableObjectsActionGoal : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef actionlib_msgs::GoalID _goal_id_type;
      _goal_id_type goal_id;
      typedef grasping_msgs::FindGraspableObjectsGoal _goal_type;
      _goal_type goal;

    FindGraspableObjectsActionGoal():
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

    virtual const char * getType() override { return "grasping_msgs/FindGraspableObjectsActionGoal"; };
    virtual const char * getMD5() override { return "8308965ee8ae423cd6ede0a494bc6f1e"; };

  };

}
#endif
