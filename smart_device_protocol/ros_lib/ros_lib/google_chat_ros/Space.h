#ifndef _ROS_google_chat_ros_Space_h
#define _ROS_google_chat_ros_Space_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace google_chat_ros
{

  class Space : public ros::Msg
  {
    public:
      typedef const char* _name_type;
      _name_type name;
      typedef const char* _display_name_type;
      _display_name_type display_name;
      typedef bool _room_type;
      _room_type room;
      typedef bool _dm_type;
      _dm_type dm;

    Space():
      name(""),
      display_name(""),
      room(0),
      dm(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      uint32_t length_name = strlen(this->name);
      varToArr(outbuffer + offset, length_name);
      offset += 4;
      memcpy(outbuffer + offset, this->name, length_name);
      offset += length_name;
      uint32_t length_display_name = strlen(this->display_name);
      varToArr(outbuffer + offset, length_display_name);
      offset += 4;
      memcpy(outbuffer + offset, this->display_name, length_display_name);
      offset += length_display_name;
      union {
        bool real;
        uint8_t base;
      } u_room;
      u_room.real = this->room;
      *(outbuffer + offset + 0) = (u_room.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->room);
      union {
        bool real;
        uint8_t base;
      } u_dm;
      u_dm.real = this->dm;
      *(outbuffer + offset + 0) = (u_dm.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->dm);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      uint32_t length_name;
      arrToVar(length_name, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_name; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_name-1]=0;
      this->name = (char *)(inbuffer + offset-1);
      offset += length_name;
      uint32_t length_display_name;
      arrToVar(length_display_name, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_display_name; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_display_name-1]=0;
      this->display_name = (char *)(inbuffer + offset-1);
      offset += length_display_name;
      union {
        bool real;
        uint8_t base;
      } u_room;
      u_room.base = 0;
      u_room.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->room = u_room.real;
      offset += sizeof(this->room);
      union {
        bool real;
        uint8_t base;
      } u_dm;
      u_dm.base = 0;
      u_dm.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->dm = u_dm.real;
      offset += sizeof(this->dm);
     return offset;
    }

    virtual const char * getType() override { return "google_chat_ros/Space"; };
    virtual const char * getMD5() override { return "8f448d7cd982f2c3d68173cc206e522d"; };

  };

}
#endif
