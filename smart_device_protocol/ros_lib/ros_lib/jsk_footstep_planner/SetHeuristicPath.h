#ifndef _ROS_SERVICE_SetHeuristicPath_h
#define _ROS_SERVICE_SetHeuristicPath_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "geometry_msgs/Point.h"

namespace jsk_footstep_planner
{

static const char SETHEURISTICPATH[] = "jsk_footstep_planner/SetHeuristicPath";

  class SetHeuristicPathRequest : public ros::Msg
  {
    public:
      uint32_t segments_length;
      typedef geometry_msgs::Point _segments_type;
      _segments_type st_segments;
      _segments_type * segments;

    SetHeuristicPathRequest():
      segments_length(0), st_segments(), segments(nullptr)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->segments_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->segments_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->segments_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->segments_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->segments_length);
      for( uint32_t i = 0; i < segments_length; i++){
      offset += this->segments[i].serialize(outbuffer + offset);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      uint32_t segments_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      segments_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      segments_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      segments_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->segments_length);
      if(segments_lengthT > segments_length)
        this->segments = (geometry_msgs::Point*)realloc(this->segments, segments_lengthT * sizeof(geometry_msgs::Point));
      segments_length = segments_lengthT;
      for( uint32_t i = 0; i < segments_length; i++){
      offset += this->st_segments.deserialize(inbuffer + offset);
        memcpy( &(this->segments[i]), &(this->st_segments), sizeof(geometry_msgs::Point));
      }
     return offset;
    }

    virtual const char * getType() override { return SETHEURISTICPATH; };
    virtual const char * getMD5() override { return "622321b353cfaa79892ba07be3ea7d2e"; };

  };

  class SetHeuristicPathResponse : public ros::Msg
  {
    public:

    SetHeuristicPathResponse()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
     return offset;
    }

    virtual const char * getType() override { return SETHEURISTICPATH; };
    virtual const char * getMD5() override { return "d41d8cd98f00b204e9800998ecf8427e"; };

  };

  class SetHeuristicPath {
    public:
    typedef SetHeuristicPathRequest Request;
    typedef SetHeuristicPathResponse Response;
  };

}
#endif
