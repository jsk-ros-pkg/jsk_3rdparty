#ifndef _ROS_moveit_msgs_OrientationConstraint_h
#define _ROS_moveit_msgs_OrientationConstraint_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "geometry_msgs/Quaternion.h"

namespace moveit_msgs
{

  class OrientationConstraint : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef geometry_msgs::Quaternion _orientation_type;
      _orientation_type orientation;
      typedef const char* _link_name_type;
      _link_name_type link_name;
      typedef float _absolute_x_axis_tolerance_type;
      _absolute_x_axis_tolerance_type absolute_x_axis_tolerance;
      typedef float _absolute_y_axis_tolerance_type;
      _absolute_y_axis_tolerance_type absolute_y_axis_tolerance;
      typedef float _absolute_z_axis_tolerance_type;
      _absolute_z_axis_tolerance_type absolute_z_axis_tolerance;
      typedef uint8_t _parameterization_type;
      _parameterization_type parameterization;
      typedef float _weight_type;
      _weight_type weight;
      enum { XYZ_EULER_ANGLES = 0 };
      enum { ROTATION_VECTOR = 1 };

    OrientationConstraint():
      header(),
      orientation(),
      link_name(""),
      absolute_x_axis_tolerance(0),
      absolute_y_axis_tolerance(0),
      absolute_z_axis_tolerance(0),
      parameterization(0),
      weight(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      offset += this->orientation.serialize(outbuffer + offset);
      uint32_t length_link_name = strlen(this->link_name);
      varToArr(outbuffer + offset, length_link_name);
      offset += 4;
      memcpy(outbuffer + offset, this->link_name, length_link_name);
      offset += length_link_name;
      offset += serializeAvrFloat64(outbuffer + offset, this->absolute_x_axis_tolerance);
      offset += serializeAvrFloat64(outbuffer + offset, this->absolute_y_axis_tolerance);
      offset += serializeAvrFloat64(outbuffer + offset, this->absolute_z_axis_tolerance);
      *(outbuffer + offset + 0) = (this->parameterization >> (8 * 0)) & 0xFF;
      offset += sizeof(this->parameterization);
      offset += serializeAvrFloat64(outbuffer + offset, this->weight);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      offset += this->orientation.deserialize(inbuffer + offset);
      uint32_t length_link_name;
      arrToVar(length_link_name, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_link_name; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_link_name-1]=0;
      this->link_name = (char *)(inbuffer + offset-1);
      offset += length_link_name;
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->absolute_x_axis_tolerance));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->absolute_y_axis_tolerance));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->absolute_z_axis_tolerance));
      this->parameterization =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->parameterization);
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->weight));
     return offset;
    }

    virtual const char * getType() override { return "moveit_msgs/OrientationConstraint"; };
    virtual const char * getMD5() override { return "183479d9281e5b4f23dc584f711d8a64"; };

  };

}
#endif
