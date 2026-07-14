#ifndef _ROS_opencv_apps_FaceArray_h
#define _ROS_opencv_apps_FaceArray_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "opencv_apps/Face.h"

namespace opencv_apps
{

  class FaceArray : public ros::Msg
  {
    public:
      uint32_t faces_length;
      typedef opencv_apps::Face _faces_type;
      _faces_type st_faces;
      _faces_type * faces;

    FaceArray():
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
        this->faces = (opencv_apps::Face*)realloc(this->faces, faces_lengthT * sizeof(opencv_apps::Face));
      faces_length = faces_lengthT;
      for( uint32_t i = 0; i < faces_length; i++){
      offset += this->st_faces.deserialize(inbuffer + offset);
        memcpy( &(this->faces[i]), &(this->st_faces), sizeof(opencv_apps::Face));
      }
     return offset;
    }

    virtual const char * getType() override { return "opencv_apps/FaceArray"; };
    virtual const char * getMD5() override { return "3ae7a36ff47d72f5dd1d764612b2b3c8"; };

  };

}
#endif
