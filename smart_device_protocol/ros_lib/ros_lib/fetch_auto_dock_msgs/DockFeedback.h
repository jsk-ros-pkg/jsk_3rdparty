#ifndef _ROS_fetch_auto_dock_msgs_DockFeedback_h
#define _ROS_fetch_auto_dock_msgs_DockFeedback_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Twist.h"

namespace fetch_auto_dock_msgs
{

  class DockFeedback : public ros::Msg
  {
    public:
      typedef geometry_msgs::PoseStamped _dock_pose_type;
      _dock_pose_type dock_pose;
      typedef geometry_msgs::Twist _command_type;
      _command_type command;

    DockFeedback():
      dock_pose(),
      command()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->dock_pose.serialize(outbuffer + offset);
      offset += this->command.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += this->dock_pose.deserialize(inbuffer + offset);
      offset += this->command.deserialize(inbuffer + offset);
     return offset;
    }

    virtual const char * getType() override { return "fetch_auto_dock_msgs/DockFeedback"; };
    virtual const char * getMD5() override { return "c91416905ed1b41536bce4f154e2b284"; };

  };

}
#endif
