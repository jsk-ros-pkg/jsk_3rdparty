#ifndef _ROS_google_chat_ros_CardHeader_h
#define _ROS_google_chat_ros_CardHeader_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace google_chat_ros
{

  class CardHeader : public ros::Msg
  {
    public:
      typedef const char* _title_type;
      _title_type title;
      typedef const char* _subtitle_type;
      _subtitle_type subtitle;
      typedef bool _image_style_circular_type;
      _image_style_circular_type image_style_circular;
      typedef const char* _image_url_type;
      _image_url_type image_url;
      typedef const char* _image_filepath_type;
      _image_filepath_type image_filepath;

    CardHeader():
      title(""),
      subtitle(""),
      image_style_circular(0),
      image_url(""),
      image_filepath("")
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      uint32_t length_title = strlen(this->title);
      varToArr(outbuffer + offset, length_title);
      offset += 4;
      memcpy(outbuffer + offset, this->title, length_title);
      offset += length_title;
      uint32_t length_subtitle = strlen(this->subtitle);
      varToArr(outbuffer + offset, length_subtitle);
      offset += 4;
      memcpy(outbuffer + offset, this->subtitle, length_subtitle);
      offset += length_subtitle;
      union {
        bool real;
        uint8_t base;
      } u_image_style_circular;
      u_image_style_circular.real = this->image_style_circular;
      *(outbuffer + offset + 0) = (u_image_style_circular.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->image_style_circular);
      uint32_t length_image_url = strlen(this->image_url);
      varToArr(outbuffer + offset, length_image_url);
      offset += 4;
      memcpy(outbuffer + offset, this->image_url, length_image_url);
      offset += length_image_url;
      uint32_t length_image_filepath = strlen(this->image_filepath);
      varToArr(outbuffer + offset, length_image_filepath);
      offset += 4;
      memcpy(outbuffer + offset, this->image_filepath, length_image_filepath);
      offset += length_image_filepath;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      uint32_t length_title;
      arrToVar(length_title, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_title; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_title-1]=0;
      this->title = (char *)(inbuffer + offset-1);
      offset += length_title;
      uint32_t length_subtitle;
      arrToVar(length_subtitle, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_subtitle; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_subtitle-1]=0;
      this->subtitle = (char *)(inbuffer + offset-1);
      offset += length_subtitle;
      union {
        bool real;
        uint8_t base;
      } u_image_style_circular;
      u_image_style_circular.base = 0;
      u_image_style_circular.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->image_style_circular = u_image_style_circular.real;
      offset += sizeof(this->image_style_circular);
      uint32_t length_image_url;
      arrToVar(length_image_url, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_image_url; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_image_url-1]=0;
      this->image_url = (char *)(inbuffer + offset-1);
      offset += length_image_url;
      uint32_t length_image_filepath;
      arrToVar(length_image_filepath, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_image_filepath; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_image_filepath-1]=0;
      this->image_filepath = (char *)(inbuffer + offset-1);
      offset += length_image_filepath;
     return offset;
    }

    virtual const char * getType() override { return "google_chat_ros/CardHeader"; };
    virtual const char * getMD5() override { return "0736280013717df387c39c7113faf889"; };

  };

}
#endif
