#ifndef _ROS_google_chat_ros_Image_h
#define _ROS_google_chat_ros_Image_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "google_chat_ros/OnClick.h"

namespace google_chat_ros
{

  class Image : public ros::Msg
  {
    public:
      typedef const char* _image_url_type;
      _image_url_type image_url;
      typedef const char* _localpath_type;
      _localpath_type localpath;
      typedef google_chat_ros::OnClick _on_click_type;
      _on_click_type on_click;
      typedef float _aspect_ratio_type;
      _aspect_ratio_type aspect_ratio;

    Image():
      image_url(""),
      localpath(""),
      on_click(),
      aspect_ratio(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      uint32_t length_image_url = strlen(this->image_url);
      varToArr(outbuffer + offset, length_image_url);
      offset += 4;
      memcpy(outbuffer + offset, this->image_url, length_image_url);
      offset += length_image_url;
      uint32_t length_localpath = strlen(this->localpath);
      varToArr(outbuffer + offset, length_localpath);
      offset += 4;
      memcpy(outbuffer + offset, this->localpath, length_localpath);
      offset += length_localpath;
      offset += this->on_click.serialize(outbuffer + offset);
      offset += serializeAvrFloat64(outbuffer + offset, this->aspect_ratio);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      uint32_t length_image_url;
      arrToVar(length_image_url, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_image_url; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_image_url-1]=0;
      this->image_url = (char *)(inbuffer + offset-1);
      offset += length_image_url;
      uint32_t length_localpath;
      arrToVar(length_localpath, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_localpath; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_localpath-1]=0;
      this->localpath = (char *)(inbuffer + offset-1);
      offset += length_localpath;
      offset += this->on_click.deserialize(inbuffer + offset);
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->aspect_ratio));
     return offset;
    }

    virtual const char * getType() override { return "google_chat_ros/Image"; };
    virtual const char * getMD5() override { return "4ca7f9f879afc606650793149935a08e"; };

  };

}
#endif
