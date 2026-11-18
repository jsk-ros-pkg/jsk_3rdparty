#ifndef _ROS_google_chat_ros_KeyValue_h
#define _ROS_google_chat_ros_KeyValue_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "google_chat_ros/OnClick.h"
#include "google_chat_ros/Button.h"

namespace google_chat_ros
{

  class KeyValue : public ros::Msg
  {
    public:
      typedef const char* _top_label_type;
      _top_label_type top_label;
      typedef const char* _content_type;
      _content_type content;
      typedef bool _content_multiline_type;
      _content_multiline_type content_multiline;
      typedef const char* _bottom_label_type;
      _bottom_label_type bottom_label;
      typedef google_chat_ros::OnClick _on_click_type;
      _on_click_type on_click;
      typedef const char* _icon_type;
      _icon_type icon;
      typedef const char* _original_icon_url_type;
      _original_icon_url_type original_icon_url;
      typedef const char* _original_icon_localpath_type;
      _original_icon_localpath_type original_icon_localpath;
      typedef google_chat_ros::Button _button_type;
      _button_type button;

    KeyValue():
      top_label(""),
      content(""),
      content_multiline(0),
      bottom_label(""),
      on_click(),
      icon(""),
      original_icon_url(""),
      original_icon_localpath(""),
      button()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      uint32_t length_top_label = strlen(this->top_label);
      varToArr(outbuffer + offset, length_top_label);
      offset += 4;
      memcpy(outbuffer + offset, this->top_label, length_top_label);
      offset += length_top_label;
      uint32_t length_content = strlen(this->content);
      varToArr(outbuffer + offset, length_content);
      offset += 4;
      memcpy(outbuffer + offset, this->content, length_content);
      offset += length_content;
      union {
        bool real;
        uint8_t base;
      } u_content_multiline;
      u_content_multiline.real = this->content_multiline;
      *(outbuffer + offset + 0) = (u_content_multiline.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->content_multiline);
      uint32_t length_bottom_label = strlen(this->bottom_label);
      varToArr(outbuffer + offset, length_bottom_label);
      offset += 4;
      memcpy(outbuffer + offset, this->bottom_label, length_bottom_label);
      offset += length_bottom_label;
      offset += this->on_click.serialize(outbuffer + offset);
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
      uint32_t length_original_icon_localpath = strlen(this->original_icon_localpath);
      varToArr(outbuffer + offset, length_original_icon_localpath);
      offset += 4;
      memcpy(outbuffer + offset, this->original_icon_localpath, length_original_icon_localpath);
      offset += length_original_icon_localpath;
      offset += this->button.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      uint32_t length_top_label;
      arrToVar(length_top_label, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_top_label; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_top_label-1]=0;
      this->top_label = (char *)(inbuffer + offset-1);
      offset += length_top_label;
      uint32_t length_content;
      arrToVar(length_content, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_content; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_content-1]=0;
      this->content = (char *)(inbuffer + offset-1);
      offset += length_content;
      union {
        bool real;
        uint8_t base;
      } u_content_multiline;
      u_content_multiline.base = 0;
      u_content_multiline.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->content_multiline = u_content_multiline.real;
      offset += sizeof(this->content_multiline);
      uint32_t length_bottom_label;
      arrToVar(length_bottom_label, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_bottom_label; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_bottom_label-1]=0;
      this->bottom_label = (char *)(inbuffer + offset-1);
      offset += length_bottom_label;
      offset += this->on_click.deserialize(inbuffer + offset);
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
      uint32_t length_original_icon_localpath;
      arrToVar(length_original_icon_localpath, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_original_icon_localpath; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_original_icon_localpath-1]=0;
      this->original_icon_localpath = (char *)(inbuffer + offset-1);
      offset += length_original_icon_localpath;
      offset += this->button.deserialize(inbuffer + offset);
     return offset;
    }

    virtual const char * getType() override { return "google_chat_ros/KeyValue"; };
    virtual const char * getMD5() override { return "b6a881cbd28397b27cef176bc48d6879"; };

  };

}
#endif
