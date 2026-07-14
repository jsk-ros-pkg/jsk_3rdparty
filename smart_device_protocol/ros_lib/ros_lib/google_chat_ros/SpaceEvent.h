#ifndef _ROS_google_chat_ros_SpaceEvent_h
#define _ROS_google_chat_ros_SpaceEvent_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "google_chat_ros/Space.h"
#include "google_chat_ros/User.h"

namespace google_chat_ros
{

  class SpaceEvent : public ros::Msg
  {
    public:
      typedef bool _added_type;
      _added_type added;
      typedef bool _removed_type;
      _removed_type removed;
      typedef const char* _event_time_type;
      _event_time_type event_time;
      typedef google_chat_ros::Space _space_type;
      _space_type space;
      typedef google_chat_ros::User _user_type;
      _user_type user;

    SpaceEvent():
      added(0),
      removed(0),
      event_time(""),
      space(),
      user()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_added;
      u_added.real = this->added;
      *(outbuffer + offset + 0) = (u_added.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->added);
      union {
        bool real;
        uint8_t base;
      } u_removed;
      u_removed.real = this->removed;
      *(outbuffer + offset + 0) = (u_removed.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->removed);
      uint32_t length_event_time = strlen(this->event_time);
      varToArr(outbuffer + offset, length_event_time);
      offset += 4;
      memcpy(outbuffer + offset, this->event_time, length_event_time);
      offset += length_event_time;
      offset += this->space.serialize(outbuffer + offset);
      offset += this->user.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_added;
      u_added.base = 0;
      u_added.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->added = u_added.real;
      offset += sizeof(this->added);
      union {
        bool real;
        uint8_t base;
      } u_removed;
      u_removed.base = 0;
      u_removed.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->removed = u_removed.real;
      offset += sizeof(this->removed);
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
      offset += this->user.deserialize(inbuffer + offset);
     return offset;
    }

    virtual const char * getType() override { return "google_chat_ros/SpaceEvent"; };
    virtual const char * getMD5() override { return "3987589bfc8301d7f07c0b4a98b57eab"; };

  };

}
#endif
