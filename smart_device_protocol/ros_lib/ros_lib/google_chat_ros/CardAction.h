#ifndef _ROS_google_chat_ros_CardAction_h
#define _ROS_google_chat_ros_CardAction_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "google_chat_ros/OnClick.h"

namespace google_chat_ros
{

  class CardAction : public ros::Msg
  {
    public:
      typedef const char* _action_label_type;
      _action_label_type action_label;
      typedef google_chat_ros::OnClick _on_click_type;
      _on_click_type on_click;

    CardAction():
      action_label(""),
      on_click()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      uint32_t length_action_label = strlen(this->action_label);
      varToArr(outbuffer + offset, length_action_label);
      offset += 4;
      memcpy(outbuffer + offset, this->action_label, length_action_label);
      offset += length_action_label;
      offset += this->on_click.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      uint32_t length_action_label;
      arrToVar(length_action_label, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_action_label; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_action_label-1]=0;
      this->action_label = (char *)(inbuffer + offset-1);
      offset += length_action_label;
      offset += this->on_click.deserialize(inbuffer + offset);
     return offset;
    }

    virtual const char * getType() override { return "google_chat_ros/CardAction"; };
    virtual const char * getMD5() override { return "c02c03a84bb896279f275f3e8c1bc1fe"; };

  };

}
#endif
