#ifndef _ROS_jsk_interactive_marker_MoveObject_h
#define _ROS_jsk_interactive_marker_MoveObject_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Pose.h"

namespace jsk_interactive_marker
{

  class MoveObject : public ros::Msg
  {
    public:
      typedef geometry_msgs::PoseStamped _origin_type;
      _origin_type origin;
      typedef geometry_msgs::PoseStamped _goal_type;
      _goal_type goal;
      typedef geometry_msgs::Pose _graspPose_type;
      _graspPose_type graspPose;

    MoveObject():
      origin(),
      goal(),
      graspPose()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->origin.serialize(outbuffer + offset);
      offset += this->goal.serialize(outbuffer + offset);
      offset += this->graspPose.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += this->origin.deserialize(inbuffer + offset);
      offset += this->goal.deserialize(inbuffer + offset);
      offset += this->graspPose.deserialize(inbuffer + offset);
     return offset;
    }

    virtual const char * getType() override { return "jsk_interactive_marker/MoveObject"; };
    virtual const char * getMD5() override { return "398be7a942bfa9cfc119a5f96e3dc37a"; };

  };

}
#endif
