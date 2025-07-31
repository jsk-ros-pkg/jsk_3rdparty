#ifndef _ROS_google_chat_ros_SendMessageGoal_h
#define _ROS_google_chat_ros_SendMessageGoal_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "google_chat_ros/Card.h"

namespace google_chat_ros
{

  class SendMessageGoal : public ros::Msg
  {
    public:
      typedef const char* _text_type;
      _text_type text;
      uint32_t cards_length;
      typedef google_chat_ros::Card _cards_type;
      _cards_type st_cards;
      _cards_type * cards;
      typedef bool _update_message_type;
      _update_message_type update_message;
      typedef const char* _thread_name_type;
      _thread_name_type thread_name;
      typedef const char* _space_type;
      _space_type space;

    SendMessageGoal():
      text(""),
      cards_length(0), st_cards(), cards(nullptr),
      update_message(0),
      thread_name(""),
      space("")
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      uint32_t length_text = strlen(this->text);
      varToArr(outbuffer + offset, length_text);
      offset += 4;
      memcpy(outbuffer + offset, this->text, length_text);
      offset += length_text;
      *(outbuffer + offset + 0) = (this->cards_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->cards_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->cards_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->cards_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->cards_length);
      for( uint32_t i = 0; i < cards_length; i++){
      offset += this->cards[i].serialize(outbuffer + offset);
      }
      union {
        bool real;
        uint8_t base;
      } u_update_message;
      u_update_message.real = this->update_message;
      *(outbuffer + offset + 0) = (u_update_message.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->update_message);
      uint32_t length_thread_name = strlen(this->thread_name);
      varToArr(outbuffer + offset, length_thread_name);
      offset += 4;
      memcpy(outbuffer + offset, this->thread_name, length_thread_name);
      offset += length_thread_name;
      uint32_t length_space = strlen(this->space);
      varToArr(outbuffer + offset, length_space);
      offset += 4;
      memcpy(outbuffer + offset, this->space, length_space);
      offset += length_space;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      uint32_t length_text;
      arrToVar(length_text, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_text; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_text-1]=0;
      this->text = (char *)(inbuffer + offset-1);
      offset += length_text;
      uint32_t cards_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      cards_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      cards_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      cards_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->cards_length);
      if(cards_lengthT > cards_length)
        this->cards = (google_chat_ros::Card*)realloc(this->cards, cards_lengthT * sizeof(google_chat_ros::Card));
      cards_length = cards_lengthT;
      for( uint32_t i = 0; i < cards_length; i++){
      offset += this->st_cards.deserialize(inbuffer + offset);
        memcpy( &(this->cards[i]), &(this->st_cards), sizeof(google_chat_ros::Card));
      }
      union {
        bool real;
        uint8_t base;
      } u_update_message;
      u_update_message.base = 0;
      u_update_message.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->update_message = u_update_message.real;
      offset += sizeof(this->update_message);
      uint32_t length_thread_name;
      arrToVar(length_thread_name, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_thread_name; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_thread_name-1]=0;
      this->thread_name = (char *)(inbuffer + offset-1);
      offset += length_thread_name;
      uint32_t length_space;
      arrToVar(length_space, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_space; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_space-1]=0;
      this->space = (char *)(inbuffer + offset-1);
      offset += length_space;
     return offset;
    }

    virtual const char * getType() override { return "google_chat_ros/SendMessageGoal"; };
    virtual const char * getMD5() override { return "228383d98cf45717fe53712a5be9ee06"; };

  };

}
#endif
