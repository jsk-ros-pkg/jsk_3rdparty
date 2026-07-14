#ifndef _ROS_google_chat_ros_Card_h
#define _ROS_google_chat_ros_Card_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "google_chat_ros/CardHeader.h"
#include "google_chat_ros/Section.h"
#include "google_chat_ros/CardAction.h"

namespace google_chat_ros
{

  class Card : public ros::Msg
  {
    public:
      typedef google_chat_ros::CardHeader _header_type;
      _header_type header;
      uint32_t sections_length;
      typedef google_chat_ros::Section _sections_type;
      _sections_type st_sections;
      _sections_type * sections;
      uint32_t card_actions_length;
      typedef google_chat_ros::CardAction _card_actions_type;
      _card_actions_type st_card_actions;
      _card_actions_type * card_actions;
      typedef const char* _name_type;
      _name_type name;

    Card():
      header(),
      sections_length(0), st_sections(), sections(nullptr),
      card_actions_length(0), st_card_actions(), card_actions(nullptr),
      name("")
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      *(outbuffer + offset + 0) = (this->sections_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->sections_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->sections_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->sections_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->sections_length);
      for( uint32_t i = 0; i < sections_length; i++){
      offset += this->sections[i].serialize(outbuffer + offset);
      }
      *(outbuffer + offset + 0) = (this->card_actions_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->card_actions_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->card_actions_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->card_actions_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->card_actions_length);
      for( uint32_t i = 0; i < card_actions_length; i++){
      offset += this->card_actions[i].serialize(outbuffer + offset);
      }
      uint32_t length_name = strlen(this->name);
      varToArr(outbuffer + offset, length_name);
      offset += 4;
      memcpy(outbuffer + offset, this->name, length_name);
      offset += length_name;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      uint32_t sections_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      sections_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      sections_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      sections_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->sections_length);
      if(sections_lengthT > sections_length)
        this->sections = (google_chat_ros::Section*)realloc(this->sections, sections_lengthT * sizeof(google_chat_ros::Section));
      sections_length = sections_lengthT;
      for( uint32_t i = 0; i < sections_length; i++){
      offset += this->st_sections.deserialize(inbuffer + offset);
        memcpy( &(this->sections[i]), &(this->st_sections), sizeof(google_chat_ros::Section));
      }
      uint32_t card_actions_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      card_actions_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      card_actions_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      card_actions_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->card_actions_length);
      if(card_actions_lengthT > card_actions_length)
        this->card_actions = (google_chat_ros::CardAction*)realloc(this->card_actions, card_actions_lengthT * sizeof(google_chat_ros::CardAction));
      card_actions_length = card_actions_lengthT;
      for( uint32_t i = 0; i < card_actions_length; i++){
      offset += this->st_card_actions.deserialize(inbuffer + offset);
        memcpy( &(this->card_actions[i]), &(this->st_card_actions), sizeof(google_chat_ros::CardAction));
      }
      uint32_t length_name;
      arrToVar(length_name, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_name; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_name-1]=0;
      this->name = (char *)(inbuffer + offset-1);
      offset += length_name;
     return offset;
    }

    virtual const char * getType() override { return "google_chat_ros/Card"; };
    virtual const char * getMD5() override { return "bb2b7c6880601ea58b79855bc615f5d1"; };

  };

}
#endif
