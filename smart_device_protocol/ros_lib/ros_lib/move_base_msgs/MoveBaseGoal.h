#ifndef _ROS_move_base_msgs_MoveBaseGoal_h
#define _ROS_move_base_msgs_MoveBaseGoal_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "geometry_msgs/PoseStamped.h"

namespace move_base_msgs
{

  class MoveBaseGoal : public ros::Msg
  {
    public:
      typedef geometry_msgs::PoseStamped _target_pose_type;
      _target_pose_type target_pose;

    MoveBaseGoal():
      target_pose()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->target_pose.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += this->target_pose.deserialize(inbuffer + offset);
     return offset;
    }

    virtual const char * getType() override { return "move_base_msgs/MoveBaseGoal"; };
    virtual const char * getMD5() override { return "257d089627d7eb7136c24d3593d05a16"; };

  };

}
#endif
