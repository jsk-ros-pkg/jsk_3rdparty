#ifndef _ROS_rtcm_msgs_Message_h
#define _ROS_rtcm_msgs_Message_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"

namespace rtcm_msgs
{

  class Message : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      uint32_t message_length;
      typedef uint8_t _message_type;
      _message_type st_message;
      _message_type * message;

    Message():
      header(),
      message_length(0), st_message(), message(nullptr)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      *(outbuffer + offset + 0) = (this->message_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->message_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->message_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->message_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->message_length);
      for( uint32_t i = 0; i < message_length; i++){
      *(outbuffer + offset + 0) = (this->message[i] >> (8 * 0)) & 0xFF;
      offset += sizeof(this->message[i]);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      uint32_t message_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      message_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      message_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      message_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->message_length);
      if(message_lengthT > message_length)
        this->message = (uint8_t*)realloc(this->message, message_lengthT * sizeof(uint8_t));
      message_length = message_lengthT;
      for( uint32_t i = 0; i < message_length; i++){
      this->st_message =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->st_message);
        memcpy( &(this->message[i]), &(this->st_message), sizeof(uint8_t));
      }
     return offset;
    }

    virtual const char * getType() override { return "rtcm_msgs/Message"; };
    virtual const char * getMD5() override { return "883b1fb65b83ccf75497c21f2d63052d"; };

  };

}
#endif
