#ifndef _ROS_google_chat_ros_MessageEvent_h
#define _ROS_google_chat_ros_MessageEvent_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "google_chat_ros/Space.h"
#include "google_chat_ros/Message.h"
#include "google_chat_ros/User.h"

namespace google_chat_ros
{

  class MessageEvent : public ros::Msg
  {
    public:
      typedef const char* _event_time_type;
      _event_time_type event_time;
      typedef google_chat_ros::Space _space_type;
      _space_type space;
      typedef google_chat_ros::Message _message_type;
      _message_type message;
      typedef google_chat_ros::User _user_type;
      _user_type user;

    MessageEvent():
      event_time(""),
      space(),
      message(),
      user()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      uint32_t length_event_time = strlen(this->event_time);
      varToArr(outbuffer + offset, length_event_time);
      offset += 4;
      memcpy(outbuffer + offset, this->event_time, length_event_time);
      offset += length_event_time;
      offset += this->space.serialize(outbuffer + offset);
      offset += this->message.serialize(outbuffer + offset);
      offset += this->user.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      uint32_t length_event_time;
      arrToVar(length_event_time, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_event_time; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_event_time-1]=0;
      this->event_time = (char *)(inbuffer + offset-1);
      offset += length_event_time;
      offset += this->space.deserialize(inbuffer + offset);
      offset += this->message.deserialize(inbuffer + offset);
      offset += this->user.deserialize(inbuffer + offset);
     return offset;
    }

    virtual const char * getType() override { return "google_chat_ros/MessageEvent"; };
    virtual const char * getMD5() override { return "e98b007d5ecc221778f8416b69f7f901"; };

  };

}
#endif
