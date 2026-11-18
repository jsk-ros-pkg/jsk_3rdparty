#ifndef _ROS_opencv_apps_Face_h
#define _ROS_opencv_apps_Face_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "opencv_apps/Rect.h"

namespace opencv_apps
{

  class Face : public ros::Msg
  {
    public:
      typedef opencv_apps::Rect _face_type;
      _face_type face;
      uint32_t eyes_length;
      typedef opencv_apps::Rect _eyes_type;
      _eyes_type st_eyes;
      _eyes_type * eyes;
      typedef const char* _label_type;
      _label_type label;
      typedef float _confidence_type;
      _confidence_type confidence;

    Face():
      face(),
      eyes_length(0), st_eyes(), eyes(nullptr),
      label(""),
      confidence(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->face.serialize(outbuffer + offset);
      *(outbuffer + offset + 0) = (this->eyes_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->eyes_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->eyes_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->eyes_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->eyes_length);
      for( uint32_t i = 0; i < eyes_length; i++){
      offset += this->eyes[i].serialize(outbuffer + offset);
      }
      uint32_t length_label = strlen(this->label);
      varToArr(outbuffer + offset, length_label);
      offset += 4;
      memcpy(outbuffer + offset, this->label, length_label);
      offset += length_label;
      offset += serializeAvrFloat64(outbuffer + offset, this->confidence);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += this->face.deserialize(inbuffer + offset);
      uint32_t eyes_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      eyes_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      eyes_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      eyes_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->eyes_length);
      if(eyes_lengthT > eyes_length)
        this->eyes = (opencv_apps::Rect*)realloc(this->eyes, eyes_lengthT * sizeof(opencv_apps::Rect));
      eyes_length = eyes_lengthT;
      for( uint32_t i = 0; i < eyes_length; i++){
      offset += this->st_eyes.deserialize(inbuffer + offset);
        memcpy( &(this->eyes[i]), &(this->st_eyes), sizeof(opencv_apps::Rect));
      }
      uint32_t length_label;
      arrToVar(length_label, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_label; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_label-1]=0;
      this->label = (char *)(inbuffer + offset-1);
      offset += length_label;
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->confidence));
     return offset;
    }

    virtual const char * getType() override { return "opencv_apps/Face"; };
    virtual const char * getMD5() override { return "a1a50e747b0ca7822ce8611c3ffa7a02"; };

  };

}
#endif
