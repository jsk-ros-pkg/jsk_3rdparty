#ifndef _ROS_jsk_interactive_marker_MarkerPose_h
#define _ROS_jsk_interactive_marker_MarkerPose_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "geometry_msgs/PoseStamped.h"

namespace jsk_interactive_marker
{

  class MarkerPose : public ros::Msg
  {
    public:
      typedef int8_t _type_type;
      _type_type type;
      typedef geometry_msgs::PoseStamped _pose_type;
      _pose_type pose;
      typedef const char* _marker_name_type;
      _marker_name_type marker_name;
      enum { GENERAL = 0 };
      enum { HEAD_MARKER = 1 };
      enum { RHAND_MARKER = 2 };
      enum { LHAND_MARKER = 3 };
      enum { RLEG_MARKER = 4 };
      enum { LLEG_MARKER = 5 };
      enum { BASE_MARKER = 6 };
      enum { RFINGER_MARKER = 7 };
      enum { LFINGER_MARKER = 8 };
      enum { SPHERE_MARKER = 9 };

    MarkerPose():
      type(0),
      pose(),
      marker_name("")
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      union {
        int8_t real;
        uint8_t base;
      } u_type;
      u_type.real = this->type;
      *(outbuffer + offset + 0) = (u_type.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->type);
      offset += this->pose.serialize(outbuffer + offset);
      uint32_t length_marker_name = strlen(this->marker_name);
      varToArr(outbuffer + offset, length_marker_name);
      offset += 4;
      memcpy(outbuffer + offset, this->marker_name, length_marker_name);
      offset += length_marker_name;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      union {
        int8_t real;
        uint8_t base;
      } u_type;
      u_type.base = 0;
      u_type.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->type = u_type.real;
      offset += sizeof(this->type);
      offset += this->pose.deserialize(inbuffer + offset);
      uint32_t length_marker_name;
      arrToVar(length_marker_name, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_marker_name; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_marker_name-1]=0;
      this->marker_name = (char *)(inbuffer + offset-1);
      offset += length_marker_name;
     return offset;
    }

    virtual const char * getType() override { return "jsk_interactive_marker/MarkerPose"; };
    virtual const char * getMD5() override { return "cbb82805055f8f87cec211c5459c476c"; };

  };

}
#endif
