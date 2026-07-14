#ifndef _ROS_SERVICE_GetFacesROI_h
#define _ROS_SERVICE_GetFacesROI_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "naoqi_bridge_msgs/FaceROI.h"

namespace naoqi_bridge_msgs
{

static const char GETFACESROI[] = "naoqi_bridge_msgs/GetFacesROI";

  class GetFacesROIRequest : public ros::Msg
  {
    public:

    GetFacesROIRequest()
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

    virtual const char * getType() override { return GETFACESROI; };
    virtual const char * getMD5() override { return "d41d8cd98f00b204e9800998ecf8427e"; };

  };

  class GetFacesROIResponse : public ros::Msg
  {
    public:
      uint32_t faces_length;
      typedef naoqi_bridge_msgs::FaceROI _faces_type;
      _faces_type st_faces;
      _faces_type * faces;

    GetFacesROIResponse():
      faces_length(0), st_faces(), faces(nullptr)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->faces_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->faces_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->faces_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->faces_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->faces_length);
      for( uint32_t i = 0; i < faces_length; i++){
      offset += this->faces[i].serialize(outbuffer + offset);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      uint32_t faces_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      faces_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      faces_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      faces_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->faces_length);
      if(faces_lengthT > faces_length)
        this->faces = (naoqi_bridge_msgs::FaceROI*)realloc(this->faces, faces_lengthT * sizeof(naoqi_bridge_msgs::FaceROI));
      faces_length = faces_lengthT;
      for( uint32_t i = 0; i < faces_length; i++){
      offset += this->st_faces.deserialize(inbuffer + offset);
        memcpy( &(this->faces[i]), &(this->st_faces), sizeof(naoqi_bridge_msgs::FaceROI));
      }
     return offset;
    }

    virtual const char * getType() override { return GETFACESROI; };
    virtual const char * getMD5() override { return "7123975ed3d5d1cde8b35e6736592769"; };

  };

  class GetFacesROI {
    public:
    typedef GetFacesROIRequest Request;
    typedef GetFacesROIResponse Response;
  };

}
#endif
