#ifndef _ROS_moveit_msgs_CartesianPoint_h
#define _ROS_moveit_msgs_CartesianPoint_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Accel.h"

namespace moveit_msgs
{

  class CartesianPoint : public ros::Msg
  {
    public:
      typedef geometry_msgs::Pose _pose_type;
      _pose_type pose;
      typedef geometry_msgs::Twist _velocity_type;
      _velocity_type velocity;
      typedef geometry_msgs::Accel _acceleration_type;
      _acceleration_type acceleration;

    CartesianPoint():
      pose(),
      velocity(),
      acceleration()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->pose.serialize(outbuffer + offset);
      offset += this->velocity.serialize(outbuffer + offset);
      offset += this->acceleration.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += this->pose.deserialize(inbuffer + offset);
      offset += this->velocity.deserialize(inbuffer + offset);
      offset += this->acceleration.deserialize(inbuffer + offset);
     return offset;
    }

    virtual const char * getType() override { return "moveit_msgs/CartesianPoint"; };
    virtual const char * getMD5() override { return "d3c213cdb4382c43adbff1f2dd2cf669"; };

  };

}
#endif
