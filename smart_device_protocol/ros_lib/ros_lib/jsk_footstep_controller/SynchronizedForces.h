#ifndef _ROS_jsk_footstep_controller_SynchronizedForces_h
#define _ROS_jsk_footstep_controller_SynchronizedForces_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "geometry_msgs/WrenchStamped.h"
#include "sensor_msgs/JointState.h"
#include "geometry_msgs/PointStamped.h"

namespace jsk_footstep_controller
{

  class SynchronizedForces : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef geometry_msgs::WrenchStamped _lleg_force_type;
      _lleg_force_type lleg_force;
      typedef geometry_msgs::WrenchStamped _rleg_force_type;
      _rleg_force_type rleg_force;
      typedef sensor_msgs::JointState _joint_angles_type;
      _joint_angles_type joint_angles;
      typedef geometry_msgs::PointStamped _zmp_type;
      _zmp_type zmp;

    SynchronizedForces():
      header(),
      lleg_force(),
      rleg_force(),
      joint_angles(),
      zmp()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      offset += this->lleg_force.serialize(outbuffer + offset);
      offset += this->rleg_force.serialize(outbuffer + offset);
      offset += this->joint_angles.serialize(outbuffer + offset);
      offset += this->zmp.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      offset += this->lleg_force.deserialize(inbuffer + offset);
      offset += this->rleg_force.deserialize(inbuffer + offset);
      offset += this->joint_angles.deserialize(inbuffer + offset);
      offset += this->zmp.deserialize(inbuffer + offset);
     return offset;
    }

    virtual const char * getType() override { return "jsk_footstep_controller/SynchronizedForces"; };
    virtual const char * getMD5() override { return "9f34791d0775ccd699ccdfdb8b823128"; };

  };

}
#endif
