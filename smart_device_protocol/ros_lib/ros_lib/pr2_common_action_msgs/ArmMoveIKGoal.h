#ifndef _ROS_pr2_common_action_msgs_ArmMoveIKGoal_h
#define _ROS_pr2_common_action_msgs_ArmMoveIKGoal_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "geometry_msgs/PoseStamped.h"
#include "sensor_msgs/JointState.h"
#include "ros/duration.h"

namespace pr2_common_action_msgs
{

  class ArmMoveIKGoal : public ros::Msg
  {
    public:
      typedef geometry_msgs::PoseStamped _pose_type;
      _pose_type pose;
      typedef geometry_msgs::PoseStamped _tool_frame_type;
      _tool_frame_type tool_frame;
      typedef sensor_msgs::JointState _ik_seed_type;
      _ik_seed_type ik_seed;
      typedef ros::Duration _ik_timeout_type;
      _ik_timeout_type ik_timeout;
      typedef ros::Duration _move_duration_type;
      _move_duration_type move_duration;

    ArmMoveIKGoal():
      pose(),
      tool_frame(),
      ik_seed(),
      ik_timeout(),
      move_duration()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->pose.serialize(outbuffer + offset);
      offset += this->tool_frame.serialize(outbuffer + offset);
      offset += this->ik_seed.serialize(outbuffer + offset);
      *(outbuffer + offset + 0) = (this->ik_timeout.sec >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->ik_timeout.sec >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->ik_timeout.sec >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->ik_timeout.sec >> (8 * 3)) & 0xFF;
      offset += sizeof(this->ik_timeout.sec);
      *(outbuffer + offset + 0) = (this->ik_timeout.nsec >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->ik_timeout.nsec >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->ik_timeout.nsec >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->ik_timeout.nsec >> (8 * 3)) & 0xFF;
      offset += sizeof(this->ik_timeout.nsec);
      *(outbuffer + offset + 0) = (this->move_duration.sec >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->move_duration.sec >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->move_duration.sec >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->move_duration.sec >> (8 * 3)) & 0xFF;
      offset += sizeof(this->move_duration.sec);
      *(outbuffer + offset + 0) = (this->move_duration.nsec >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->move_duration.nsec >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->move_duration.nsec >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->move_duration.nsec >> (8 * 3)) & 0xFF;
      offset += sizeof(this->move_duration.nsec);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += this->pose.deserialize(inbuffer + offset);
      offset += this->tool_frame.deserialize(inbuffer + offset);
      offset += this->ik_seed.deserialize(inbuffer + offset);
      this->ik_timeout.sec =  ((uint32_t) (*(inbuffer + offset)));
      this->ik_timeout.sec |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->ik_timeout.sec |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->ik_timeout.sec |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->ik_timeout.sec);
      this->ik_timeout.nsec =  ((uint32_t) (*(inbuffer + offset)));
      this->ik_timeout.nsec |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->ik_timeout.nsec |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->ik_timeout.nsec |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->ik_timeout.nsec);
      this->move_duration.sec =  ((uint32_t) (*(inbuffer + offset)));
      this->move_duration.sec |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->move_duration.sec |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->move_duration.sec |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->move_duration.sec);
      this->move_duration.nsec =  ((uint32_t) (*(inbuffer + offset)));
      this->move_duration.nsec |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->move_duration.nsec |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->move_duration.nsec |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->move_duration.nsec);
     return offset;
    }

    virtual const char * getType() override { return "pr2_common_action_msgs/ArmMoveIKGoal"; };
    virtual const char * getMD5() override { return "659cdac4f142756518faf4644a34bdda"; };

  };

}
#endif
