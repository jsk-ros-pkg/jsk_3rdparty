#ifndef _ROS_google_chat_ros_User_h
#define _ROS_google_chat_ros_User_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace google_chat_ros
{

  class User : public ros::Msg
  {
    public:
      typedef const char* _name_type;
      _name_type name;
      typedef const char* _display_name_type;
      _display_name_type display_name;
      typedef const char* _avatar_url_type;
      _avatar_url_type avatar_url;
      uint32_t avatar_length;
      typedef uint8_t _avatar_type;
      _avatar_type st_avatar;
      _avatar_type * avatar;
      typedef const char* _email_type;
      _email_type email;
      typedef bool _bot_type;
      _bot_type bot;
      typedef bool _human_type;
      _human_type human;

    User():
      name(""),
      display_name(""),
      avatar_url(""),
      avatar_length(0), st_avatar(), avatar(nullptr),
      email(""),
      bot(0),
      human(0)
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
      uint32_t length_avatar_url = strlen(this->avatar_url);
      varToArr(outbuffer + offset, length_avatar_url);
      offset += 4;
      memcpy(outbuffer + offset, this->avatar_url, length_avatar_url);
      offset += length_avatar_url;
      *(outbuffer + offset + 0) = (this->avatar_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->avatar_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->avatar_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->avatar_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->avatar_length);
      for( uint32_t i = 0; i < avatar_length; i++){
      *(outbuffer + offset + 0) = (this->avatar[i] >> (8 * 0)) & 0xFF;
      offset += sizeof(this->avatar[i]);
      }
      uint32_t length_email = strlen(this->email);
      varToArr(outbuffer + offset, length_email);
      offset += 4;
      memcpy(outbuffer + offset, this->email, length_email);
      offset += length_email;
      union {
        bool real;
        uint8_t base;
      } u_bot;
      u_bot.real = this->bot;
      *(outbuffer + offset + 0) = (u_bot.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->bot);
      union {
        bool real;
        uint8_t base;
      } u_human;
      u_human.real = this->human;
      *(outbuffer + offset + 0) = (u_human.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->human);
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
      uint32_t length_avatar_url;
      arrToVar(length_avatar_url, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_avatar_url; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_avatar_url-1]=0;
      this->avatar_url = (char *)(inbuffer + offset-1);
      offset += length_avatar_url;
      uint32_t avatar_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      avatar_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      avatar_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      avatar_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->avatar_length);
      if(avatar_lengthT > avatar_length)
        this->avatar = (uint8_t*)realloc(this->avatar, avatar_lengthT * sizeof(uint8_t));
      avatar_length = avatar_lengthT;
      for( uint32_t i = 0; i < avatar_length; i++){
      this->st_avatar =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->st_avatar);
        memcpy( &(this->avatar[i]), &(this->st_avatar), sizeof(uint8_t));
      }
      uint32_t length_email;
      arrToVar(length_email, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_email; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_email-1]=0;
      this->email = (char *)(inbuffer + offset-1);
      offset += length_email;
      union {
        bool real;
        uint8_t base;
      } u_bot;
      u_bot.base = 0;
      u_bot.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->bot = u_bot.real;
      offset += sizeof(this->bot);
      union {
        bool real;
        uint8_t base;
      } u_human;
      u_human.base = 0;
      u_human.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->human = u_human.real;
      offset += sizeof(this->human);
     return offset;
    }

    virtual const char * getType() override { return "google_chat_ros/User"; };
    virtual const char * getMD5() override { return "8f7f8da1e0e6ba94d6bd196cb5950d15"; };

  };

}
#endif
