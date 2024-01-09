#ifndef _ROS_jsk_interactive_marker_SnapFootPrintInput_h
#define _ROS_jsk_interactive_marker_SnapFootPrintInput_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Pose.h"

namespace jsk_interactive_marker
{

  class SnapFootPrintInput : public ros::Msg
  {
    public:
      typedef geometry_msgs::PoseStamped _input_pose_type;
      _input_pose_type input_pose;
      typedef geometry_msgs::Pose _lleg_pose_type;
      _lleg_pose_type lleg_pose;
      typedef geometry_msgs::Pose _rleg_pose_type;
      _rleg_pose_type rleg_pose;

    SnapFootPrintInput():
      input_pose(),
      lleg_pose(),
      rleg_pose()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->input_pose.serialize(outbuffer + offset);
      offset += this->lleg_pose.serialize(outbuffer + offset);
      offset += this->rleg_pose.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += this->input_pose.deserialize(inbuffer + offset);
      offset += this->lleg_pose.deserialize(inbuffer + offset);
      offset += this->rleg_pose.deserialize(inbuffer + offset);
     return offset;
    }

    virtual const char * getType() override { return "jsk_interactive_marker/SnapFootPrintInput"; };
    virtual const char * getMD5() override { return "07fc9b79352f12bc13742f589662de86"; };

  };

}
#endif
