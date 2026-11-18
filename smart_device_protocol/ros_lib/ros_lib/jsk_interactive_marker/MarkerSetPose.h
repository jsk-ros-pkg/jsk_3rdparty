#ifndef _ROS_SERVICE_MarkerSetPose_h
#define _ROS_SERVICE_MarkerSetPose_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "geometry_msgs/PoseStamped.h"
#include "visualization_msgs/Marker.h"

namespace jsk_interactive_marker
{

static const char MARKERSETPOSE[] = "jsk_interactive_marker/MarkerSetPose";

  class MarkerSetPoseRequest : public ros::Msg
  {
    public:
      typedef const char* _marker_name_type;
      _marker_name_type marker_name;
      typedef geometry_msgs::PoseStamped _pose_type;
      _pose_type pose;
      uint32_t markers_length;
      typedef visualization_msgs::Marker _markers_type;
      _markers_type st_markers;
      _markers_type * markers;

    MarkerSetPoseRequest():
      marker_name(""),
      pose(),
      markers_length(0), st_markers(), markers(nullptr)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      uint32_t length_marker_name = strlen(this->marker_name);
      varToArr(outbuffer + offset, length_marker_name);
      offset += 4;
      memcpy(outbuffer + offset, this->marker_name, length_marker_name);
      offset += length_marker_name;
      offset += this->pose.serialize(outbuffer + offset);
      *(outbuffer + offset + 0) = (this->markers_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->markers_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->markers_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->markers_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->markers_length);
      for( uint32_t i = 0; i < markers_length; i++){
      offset += this->markers[i].serialize(outbuffer + offset);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      uint32_t length_marker_name;
      arrToVar(length_marker_name, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_marker_name; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_marker_name-1]=0;
      this->marker_name = (char *)(inbuffer + offset-1);
      offset += length_marker_name;
      offset += this->pose.deserialize(inbuffer + offset);
      uint32_t markers_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      markers_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      markers_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      markers_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->markers_length);
      if(markers_lengthT > markers_length)
        this->markers = (visualization_msgs::Marker*)realloc(this->markers, markers_lengthT * sizeof(visualization_msgs::Marker));
      markers_length = markers_lengthT;
      for( uint32_t i = 0; i < markers_length; i++){
      offset += this->st_markers.deserialize(inbuffer + offset);
        memcpy( &(this->markers[i]), &(this->st_markers), sizeof(visualization_msgs::Marker));
      }
     return offset;
    }

    virtual const char * getType() override { return MARKERSETPOSE; };
    virtual const char * getMD5() override { return "ad9cd9afd2203415fba0c5648ca2decb"; };

  };

  class MarkerSetPoseResponse : public ros::Msg
  {
    public:
      typedef geometry_msgs::PoseStamped _pose_type;
      _pose_type pose;

    MarkerSetPoseResponse():
      pose()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->pose.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += this->pose.deserialize(inbuffer + offset);
     return offset;
    }

    virtual const char * getType() override { return MARKERSETPOSE; };
    virtual const char * getMD5() override { return "3f8930d968a3e84d471dff917bb1cdae"; };

  };

  class MarkerSetPose {
    public:
    typedef MarkerSetPoseRequest Request;
    typedef MarkerSetPoseResponse Response;
  };

}
#endif
