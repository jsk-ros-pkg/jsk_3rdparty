#ifndef _ROS_graft_GraftState_h
#define _ROS_graft_GraftState_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Vector3.h"

namespace graft
{

  class GraftState : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef geometry_msgs::Pose _pose_type;
      _pose_type pose;
      typedef geometry_msgs::Twist _twist_type;
      _twist_type twist;
      typedef geometry_msgs::Vector3 _acceleration_type;
      _acceleration_type acceleration;
      typedef geometry_msgs::Vector3 _gyro_bias_type;
      _gyro_bias_type gyro_bias;
      typedef geometry_msgs::Vector3 _accel_bias_type;
      _accel_bias_type accel_bias;
      float covariance[324];

    GraftState():
      header(),
      pose(),
      twist(),
      acceleration(),
      gyro_bias(),
      accel_bias(),
      covariance()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      offset += this->pose.serialize(outbuffer + offset);
      offset += this->twist.serialize(outbuffer + offset);
      offset += this->acceleration.serialize(outbuffer + offset);
      offset += this->gyro_bias.serialize(outbuffer + offset);
      offset += this->accel_bias.serialize(outbuffer + offset);
      for( uint32_t i = 0; i < 324; i++){
      offset += serializeAvrFloat64(outbuffer + offset, this->covariance[i]);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      offset += this->pose.deserialize(inbuffer + offset);
      offset += this->twist.deserialize(inbuffer + offset);
      offset += this->acceleration.deserialize(inbuffer + offset);
      offset += this->gyro_bias.deserialize(inbuffer + offset);
      offset += this->accel_bias.deserialize(inbuffer + offset);
      for( uint32_t i = 0; i < 324; i++){
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->covariance[i]));
      }
     return offset;
    }

    virtual const char * getType() override { return "graft/GraftState"; };
    virtual const char * getMD5() override { return "4744aac037427813b68053f3a09da177"; };

  };

}
#endif
