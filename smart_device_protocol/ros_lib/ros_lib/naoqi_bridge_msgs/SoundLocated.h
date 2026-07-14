#ifndef _ROS_naoqi_bridge_msgs_SoundLocated_h
#define _ROS_naoqi_bridge_msgs_SoundLocated_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "geometry_msgs/Twist.h"

namespace naoqi_bridge_msgs
{

  class SoundLocated : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef float _azimuth_type;
      _azimuth_type azimuth;
      typedef float _elevation_type;
      _elevation_type elevation;
      typedef float _confidence_type;
      _confidence_type confidence;
      typedef float _energy_type;
      _energy_type energy;
      typedef geometry_msgs::Twist _head_position_frame_torso_type;
      _head_position_frame_torso_type head_position_frame_torso;
      typedef geometry_msgs::Twist _head_position_frame_robot_type;
      _head_position_frame_robot_type head_position_frame_robot;

    SoundLocated():
      header(),
      azimuth(0),
      elevation(0),
      confidence(0),
      energy(0),
      head_position_frame_torso(),
      head_position_frame_robot()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      offset += serializeAvrFloat64(outbuffer + offset, this->azimuth);
      offset += serializeAvrFloat64(outbuffer + offset, this->elevation);
      offset += serializeAvrFloat64(outbuffer + offset, this->confidence);
      offset += serializeAvrFloat64(outbuffer + offset, this->energy);
      offset += this->head_position_frame_torso.serialize(outbuffer + offset);
      offset += this->head_position_frame_robot.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->azimuth));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->elevation));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->confidence));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->energy));
      offset += this->head_position_frame_torso.deserialize(inbuffer + offset);
      offset += this->head_position_frame_robot.deserialize(inbuffer + offset);
     return offset;
    }

    virtual const char * getType() override { return "naoqi_bridge_msgs/SoundLocated"; };
    virtual const char * getMD5() override { return "884a2810157403bbdabfb1011c851b42"; };

  };

}
#endif
