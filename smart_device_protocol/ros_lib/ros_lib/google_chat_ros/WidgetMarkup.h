#ifndef _ROS_google_chat_ros_WidgetMarkup_h
#define _ROS_google_chat_ros_WidgetMarkup_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "google_chat_ros/Button.h"
#include "google_chat_ros/Image.h"
#include "google_chat_ros/KeyValue.h"

namespace google_chat_ros
{

  class WidgetMarkup : public ros::Msg
  {
    public:
      uint32_t buttons_length;
      typedef google_chat_ros::Button _buttons_type;
      _buttons_type st_buttons;
      _buttons_type * buttons;
      typedef const char* _text_paragraph_type;
      _text_paragraph_type text_paragraph;
      typedef google_chat_ros::Image _image_type;
      _image_type image;
      typedef google_chat_ros::KeyValue _key_value_type;
      _key_value_type key_value;

    WidgetMarkup():
      buttons_length(0), st_buttons(), buttons(nullptr),
      text_paragraph(""),
      image(),
      key_value()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->buttons_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->buttons_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->buttons_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->buttons_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->buttons_length);
      for( uint32_t i = 0; i < buttons_length; i++){
      offset += this->buttons[i].serialize(outbuffer + offset);
      }
      uint32_t length_text_paragraph = strlen(this->text_paragraph);
      varToArr(outbuffer + offset, length_text_paragraph);
      offset += 4;
      memcpy(outbuffer + offset, this->text_paragraph, length_text_paragraph);
      offset += length_text_paragraph;
      offset += this->image.serialize(outbuffer + offset);
      offset += this->key_value.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      uint32_t buttons_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      buttons_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      buttons_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      buttons_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->buttons_length);
      if(buttons_lengthT > buttons_length)
        this->buttons = (google_chat_ros::Button*)realloc(this->buttons, buttons_lengthT * sizeof(google_chat_ros::Button));
      buttons_length = buttons_lengthT;
      for( uint32_t i = 0; i < buttons_length; i++){
      offset += this->st_buttons.deserialize(inbuffer + offset);
        memcpy( &(this->buttons[i]), &(this->st_buttons), sizeof(google_chat_ros::Button));
      }
      uint32_t length_text_paragraph;
      arrToVar(length_text_paragraph, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_text_paragraph; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_text_paragraph-1]=0;
      this->text_paragraph = (char *)(inbuffer + offset-1);
      offset += length_text_paragraph;
      offset += this->image.deserialize(inbuffer + offset);
      offset += this->key_value.deserialize(inbuffer + offset);
     return offset;
    }

    virtual const char * getType() override { return "google_chat_ros/WidgetMarkup"; };
    virtual const char * getMD5() override { return "df2ae4247b1cbe16ca432a0ead1ba903"; };

  };

}
#endif
