#ifndef _ROS_mongodb_store_msgs_SerialisedMessage_h
#define _ROS_mongodb_store_msgs_SerialisedMessage_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace mongodb_store_msgs
{

  class SerialisedMessage : public ros::Msg
  {
    public:
      typedef const char* _type_type;
      _type_type type;
      uint32_t msg_length;
      typedef uint8_t _msg_type;
      _msg_type st_msg;
      _msg_type * msg;

    SerialisedMessage():
      type(""),
      msg_length(0), st_msg(), msg(nullptr)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      uint32_t length_type = strlen(this->type);
      varToArr(outbuffer + offset, length_type);
      offset += 4;
      memcpy(outbuffer + offset, this->type, length_type);
      offset += length_type;
      *(outbuffer + offset + 0) = (this->msg_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->msg_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->msg_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->msg_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->msg_length);
      for( uint32_t i = 0; i < msg_length; i++){
      *(outbuffer + offset + 0) = (this->msg[i] >> (8 * 0)) & 0xFF;
      offset += sizeof(this->msg[i]);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      uint32_t length_type;
      arrToVar(length_type, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_type; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_type-1]=0;
      this->type = (char *)(inbuffer + offset-1);
      offset += length_type;
      uint32_t msg_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      msg_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      msg_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      msg_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->msg_length);
      if(msg_lengthT > msg_length)
        this->msg = (uint8_t*)realloc(this->msg, msg_lengthT * sizeof(uint8_t));
      msg_length = msg_lengthT;
      for( uint32_t i = 0; i < msg_length; i++){
      this->st_msg =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->st_msg);
        memcpy( &(this->msg[i]), &(this->st_msg), sizeof(uint8_t));
      }
     return offset;
    }

    virtual const char * getType() override { return "mongodb_store_msgs/SerialisedMessage"; };
    virtual const char * getMD5() override { return "42f77e70b6ff70f99d1597d836874cfc"; };

  };

}
#endif
