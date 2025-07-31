#ifndef _ROS_google_chat_ros_Button_h
#define _ROS_google_chat_ros_Button_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "google_chat_ros/OnClick.h"

namespace google_chat_ros
{

  class Button : public ros::Msg
  {
    public:
      typedef const char* _text_button_name_type;
      _text_button_name_type text_button_name;
      typedef google_chat_ros::OnClick _text_button_on_click_type;
      _text_button_on_click_type text_button_on_click;
      typedef const char* _image_button_name_type;
      _image_button_name_type image_button_name;
      typedef google_chat_ros::OnClick _image_button_on_click_type;
      _image_button_on_click_type image_button_on_click;
      typedef const char* _icon_type;
      _icon_type icon;
      typedef const char* _original_icon_url_type;
      _original_icon_url_type original_icon_url;
      typedef const char* _original_icon_filepath_type;
      _original_icon_filepath_type original_icon_filepath;

    Button():
      text_button_name(""),
      text_button_on_click(),
      image_button_name(""),
      image_button_on_click(),
      icon(""),
      original_icon_url(""),
      original_icon_filepath("")
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      uint32_t length_text_button_name = strlen(this->text_button_name);
      varToArr(outbuffer + offset, length_text_button_name);
      offset += 4;
      memcpy(outbuffer + offset, this->text_button_name, length_text_button_name);
      offset += length_text_button_name;
      offset += this->text_button_on_click.serialize(outbuffer + offset);
      uint32_t length_image_button_name = strlen(this->image_button_name);
      varToArr(outbuffer + offset, length_image_button_name);
      offset += 4;
      memcpy(outbuffer + offset, this->image_button_name, length_image_button_name);
      offset += length_image_button_name;
      offset += this->image_button_on_click.serialize(outbuffer + offset);
      uint32_t length_icon = strlen(this->icon);
      varToArr(outbuffer + offset, length_icon);
      offset += 4;
      memcpy(outbuffer + offset, this->icon, length_icon);
      offset += length_icon;
      uint32_t length_original_icon_url = strlen(this->original_icon_url);
      varToArr(outbuffer + offset, length_original_icon_url);
      offset += 4;
      memcpy(outbuffer + offset, this->original_icon_url, length_original_icon_url);
      offset += length_original_icon_url;
      uint32_t length_original_icon_filepath = strlen(this->original_icon_filepath);
      varToArr(outbuffer + offset, length_original_icon_filepath);
      offset += 4;
      memcpy(outbuffer + offset, this->original_icon_filepath, length_original_icon_filepath);
      offset += length_original_icon_filepath;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      uint32_t length_text_button_name;
      arrToVar(length_text_button_name, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_text_button_name; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_text_button_name-1]=0;
      this->text_button_name = (char *)(inbuffer + offset-1);
      offset += length_text_button_name;
      offset += this->text_button_on_click.deserialize(inbuffer + offset);
      uint32_t length_image_button_name;
      arrToVar(length_image_button_name, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_image_button_name; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_image_button_name-1]=0;
      this->image_button_name = (char *)(inbuffer + offset-1);
      offset += length_image_button_name;
      offset += this->image_button_on_click.deserialize(inbuffer + offset);
      uint32_t length_icon;
      arrToVar(length_icon, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_icon; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_icon-1]=0;
      this->icon = (char *)(inbuffer + offset-1);
      offset += length_icon;
      uint32_t length_original_icon_url;
      arrToVar(length_original_icon_url, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_original_icon_url; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_original_icon_url-1]=0;
      this->original_icon_url = (char *)(inbuffer + offset-1);
      offset += length_original_icon_url;
      uint32_t length_original_icon_filepath;
      arrToVar(length_original_icon_filepath, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_original_icon_filepath; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_original_icon_filepath-1]=0;
      this->original_icon_filepath = (char *)(inbuffer + offset-1);
      offset += length_original_icon_filepath;
     return offset;
    }

    virtual const char * getType() override { return "google_chat_ros/Button"; };
    virtual const char * getMD5() override { return "d1068b3005f469e72c9f1e45775e9406"; };

  };

}
#endif
