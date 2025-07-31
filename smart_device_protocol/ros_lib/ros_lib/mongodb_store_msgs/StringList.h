#ifndef _ROS_mongodb_store_msgs_StringList_h
#define _ROS_mongodb_store_msgs_StringList_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace mongodb_store_msgs
{

  class StringList : public ros::Msg
  {
    public:
      uint32_t data_length;
      typedef char* _data_type;
      _data_type st_data;
      _data_type * data;

    StringList():
      data_length(0), st_data(), data(nullptr)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->data_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->data_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->data_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->data_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->data_length);
      for( uint32_t i = 0; i < data_length; i++){
      uint32_t length_datai = strlen(this->data[i]);
      varToArr(outbuffer + offset, length_datai);
      offset += 4;
      memcpy(outbuffer + offset, this->data[i], length_datai);
      offset += length_datai;
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      uint32_t data_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      data_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      data_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      data_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->data_length);
      if(data_lengthT > data_length)
        this->data = (char**)realloc(this->data, data_lengthT * sizeof(char*));
      data_length = data_lengthT;
      for( uint32_t i = 0; i < data_length; i++){
      uint32_t length_st_data;
      arrToVar(length_st_data, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_st_data; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_st_data-1]=0;
      this->st_data = (char *)(inbuffer + offset-1);
      offset += length_st_data;
        memcpy( &(this->data[i]), &(this->st_data), sizeof(char*));
      }
     return offset;
    }

    virtual const char * getType() override { return "mongodb_store_msgs/StringList"; };
    virtual const char * getMD5() override { return "cce5a364f3a3be12c9722c6dcad2fa94"; };

  };

}
#endif
