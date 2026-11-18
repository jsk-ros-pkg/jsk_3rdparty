#ifndef _ROS_SERVICE_SnapFootPrint_h
#define _ROS_SERVICE_SnapFootPrint_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Pose.h"

namespace jsk_interactive_marker
{

static const char SNAPFOOTPRINT[] = "jsk_interactive_marker/SnapFootPrint";

  class SnapFootPrintRequest : public ros::Msg
  {
    public:
      typedef geometry_msgs::PoseStamped _input_pose_type;
      _input_pose_type input_pose;
      typedef geometry_msgs::Pose _lleg_pose_type;
      _lleg_pose_type lleg_pose;
      typedef geometry_msgs::Pose _rleg_pose_type;
      _rleg_pose_type rleg_pose;

    SnapFootPrintRequest():
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

    virtual const char * getType() override { return SNAPFOOTPRINT; };
    virtual const char * getMD5() override { return "07fc9b79352f12bc13742f589662de86"; };

  };

  class SnapFootPrintResponse : public ros::Msg
  {
    public:
      typedef bool _success_type;
      _success_type success;
      typedef geometry_msgs::PoseStamped _snapped_pose_type;
      _snapped_pose_type snapped_pose;

    SnapFootPrintResponse():
      success(0),
      snapped_pose()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_success;
      u_success.real = this->success;
      *(outbuffer + offset + 0) = (u_success.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->success);
      offset += this->snapped_pose.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_success;
      u_success.base = 0;
      u_success.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->success = u_success.real;
      offset += sizeof(this->success);
      offset += this->snapped_pose.deserialize(inbuffer + offset);
     return offset;
    }

    virtual const char * getType() override { return SNAPFOOTPRINT; };
    virtual const char * getMD5() override { return "a900428084c91b5e6bdb3a3c62cb401d"; };

  };

  class SnapFootPrint {
    public:
    typedef SnapFootPrintRequest Request;
    typedef SnapFootPrintResponse Response;
  };

}
#endif
