#ifndef _ROS_graft_GraftSensorResidual_h
#define _ROS_graft_GraftSensorResidual_h

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

  class GraftSensorResidual : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef const char* _name_type;
      _name_type name;
      typedef geometry_msgs::Pose _pose_type;
      _pose_type pose;
      typedef geometry_msgs::Twist _twist_type;
      _twist_type twist;
      typedef geometry_msgs::Vector3 _accel_type;
      _accel_type accel;
      float pose_covariance[36];
      float twist_covariance[36];
      float accel_covariance[9];

    GraftSensorResidual():
      header(),
      name(""),
      pose(),
      twist(),
      accel(),
      pose_covariance(),
      twist_covariance(),
      accel_covariance()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      uint32_t length_name = strlen(this->name);
      varToArr(outbuffer + offset, length_name);
      offset += 4;
      memcpy(outbuffer + offset, this->name, length_name);
      offset += length_name;
      offset += this->pose.serialize(outbuffer + offset);
      offset += this->twist.serialize(outbuffer + offset);
      offset += this->accel.serialize(outbuffer + offset);
      for( uint32_t i = 0; i < 36; i++){
      offset += serializeAvrFloat64(outbuffer + offset, this->pose_covariance[i]);
      }
      for( uint32_t i = 0; i < 36; i++){
      offset += serializeAvrFloat64(outbuffer + offset, this->twist_covariance[i]);
      }
      for( uint32_t i = 0; i < 9; i++){
      offset += serializeAvrFloat64(outbuffer + offset, this->accel_covariance[i]);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      uint32_t length_name;
      arrToVar(length_name, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_name; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_name-1]=0;
      this->name = (char *)(inbuffer + offset-1);
      offset += length_name;
      offset += this->pose.deserialize(inbuffer + offset);
      offset += this->twist.deserialize(inbuffer + offset);
      offset += this->accel.deserialize(inbuffer + offset);
      for( uint32_t i = 0; i < 36; i++){
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->pose_covariance[i]));
      }
      for( uint32_t i = 0; i < 36; i++){
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->twist_covariance[i]));
      }
      for( uint32_t i = 0; i < 9; i++){
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->accel_covariance[i]));
      }
     return offset;
    }

    virtual const char * getType() override { return "graft/GraftSensorResidual"; };
    virtual const char * getMD5() override { return "f75937e1c71e90285875737addd2c780"; };

  };

}
#endif
