#ifndef _ROS_google_chat_ros_OnClick_h
#define _ROS_google_chat_ros_OnClick_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "google_chat_ros/FormAction.h"

namespace google_chat_ros
{

  class OnClick : public ros::Msg
  {
    public:
      typedef google_chat_ros::FormAction _action_type;
      _action_type action;
      typedef const char* _open_link_url_type;
      _open_link_url_type open_link_url;

    OnClick():
      action(),
      open_link_url("")
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->action.serialize(outbuffer + offset);
      uint32_t length_open_link_url = strlen(this->open_link_url);
      varToArr(outbuffer + offset, length_open_link_url);
      offset += 4;
      memcpy(outbuffer + offset, this->open_link_url, length_open_link_url);
      offset += length_open_link_url;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += this->action.deserialize(inbuffer + offset);
      uint32_t length_open_link_url;
      arrToVar(length_open_link_url, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_open_link_url; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_open_link_url-1]=0;
      this->open_link_url = (char *)(inbuffer + offset-1);
      offset += length_open_link_url;
     return offset;
    }

    virtual const char * getType() override { return "google_chat_ros/OnClick"; };
    virtual const char * getMD5() override { return "fa8b3a77f7093cce31488d398e534227"; };

  };

}
#endif
