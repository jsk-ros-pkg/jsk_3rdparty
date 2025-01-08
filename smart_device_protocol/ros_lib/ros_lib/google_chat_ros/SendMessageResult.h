#ifndef _ROS_google_chat_ros_SendMessageResult_h
#define _ROS_google_chat_ros_SendMessageResult_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "google_chat_ros/Message.h"
#include "google_chat_ros/Card.h"

namespace google_chat_ros
{

  class SendMessageResult : public ros::Msg
  {
    public:
      typedef google_chat_ros::Message _message_result_type;
      _message_result_type message_result;
      uint32_t cards_result_length;
      typedef google_chat_ros::Card _cards_result_type;
      _cards_result_type st_cards_result;
      _cards_result_type * cards_result;
      typedef bool _done_type;
      _done_type done;

    SendMessageResult():
      message_result(),
      cards_result_length(0), st_cards_result(), cards_result(nullptr),
      done(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->message_result.serialize(outbuffer + offset);
      *(outbuffer + offset + 0) = (this->cards_result_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->cards_result_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->cards_result_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->cards_result_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->cards_result_length);
      for( uint32_t i = 0; i < cards_result_length; i++){
      offset += this->cards_result[i].serialize(outbuffer + offset);
      }
      union {
        bool real;
        uint8_t base;
      } u_done;
      u_done.real = this->done;
      *(outbuffer + offset + 0) = (u_done.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->done);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += this->message_result.deserialize(inbuffer + offset);
      uint32_t cards_result_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      cards_result_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      cards_result_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      cards_result_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->cards_result_length);
      if(cards_result_lengthT > cards_result_length)
        this->cards_result = (google_chat_ros::Card*)realloc(this->cards_result, cards_result_lengthT * sizeof(google_chat_ros::Card));
      cards_result_length = cards_result_lengthT;
      for( uint32_t i = 0; i < cards_result_length; i++){
      offset += this->st_cards_result.deserialize(inbuffer + offset);
        memcpy( &(this->cards_result[i]), &(this->st_cards_result), sizeof(google_chat_ros::Card));
      }
      union {
        bool real;
        uint8_t base;
      } u_done;
      u_done.base = 0;
      u_done.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->done = u_done.real;
      offset += sizeof(this->done);
     return offset;
    }

    virtual const char * getType() override { return "google_chat_ros/SendMessageResult"; };
    virtual const char * getMD5() override { return "2debf2fb1ec68366c318ee077fe23b31"; };

  };

}
#endif
