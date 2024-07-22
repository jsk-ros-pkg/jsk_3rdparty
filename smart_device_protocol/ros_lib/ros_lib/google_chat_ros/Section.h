#ifndef _ROS_google_chat_ros_Section_h
#define _ROS_google_chat_ros_Section_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "google_chat_ros/WidgetMarkup.h"

namespace google_chat_ros
{

  class Section : public ros::Msg
  {
    public:
      typedef const char* _header_type;
      _header_type header;
      uint32_t widgets_length;
      typedef google_chat_ros::WidgetMarkup _widgets_type;
      _widgets_type st_widgets;
      _widgets_type * widgets;

    Section():
      header(""),
      widgets_length(0), st_widgets(), widgets(nullptr)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      uint32_t length_header = strlen(this->header);
      varToArr(outbuffer + offset, length_header);
      offset += 4;
      memcpy(outbuffer + offset, this->header, length_header);
      offset += length_header;
      *(outbuffer + offset + 0) = (this->widgets_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->widgets_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->widgets_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->widgets_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->widgets_length);
      for( uint32_t i = 0; i < widgets_length; i++){
      offset += this->widgets[i].serialize(outbuffer + offset);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      uint32_t length_header;
      arrToVar(length_header, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_header; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_header-1]=0;
      this->header = (char *)(inbuffer + offset-1);
      offset += length_header;
      uint32_t widgets_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      widgets_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      widgets_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      widgets_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->widgets_length);
      if(widgets_lengthT > widgets_length)
        this->widgets = (google_chat_ros::WidgetMarkup*)realloc(this->widgets, widgets_lengthT * sizeof(google_chat_ros::WidgetMarkup));
      widgets_length = widgets_lengthT;
      for( uint32_t i = 0; i < widgets_length; i++){
      offset += this->st_widgets.deserialize(inbuffer + offset);
        memcpy( &(this->widgets[i]), &(this->st_widgets), sizeof(google_chat_ros::WidgetMarkup));
      }
     return offset;
    }

    virtual const char * getType() override { return "google_chat_ros/Section"; };
    virtual const char * getMD5() override { return "90eb3f68d10d183e560f956adae61989"; };

  };

}
#endif
