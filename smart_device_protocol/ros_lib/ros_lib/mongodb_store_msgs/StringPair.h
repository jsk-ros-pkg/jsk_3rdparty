#ifndef _ROS_mongodb_store_msgs_StringPair_h
#define _ROS_mongodb_store_msgs_StringPair_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace mongodb_store_msgs
{

  class StringPair : public ros::Msg
  {
    public:
      typedef const char* _first_type;
      _first_type first;
      typedef const char* _second_type;
      _second_type second;

    StringPair():
      first(""),
      second("")
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      uint32_t length_first = strlen(this->first);
      varToArr(outbuffer + offset, length_first);
      offset += 4;
      memcpy(outbuffer + offset, this->first, length_first);
      offset += length_first;
      uint32_t length_second = strlen(this->second);
      varToArr(outbuffer + offset, length_second);
      offset += 4;
      memcpy(outbuffer + offset, this->second, length_second);
      offset += length_second;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      uint32_t length_first;
      arrToVar(length_first, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_first; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_first-1]=0;
      this->first = (char *)(inbuffer + offset-1);
      offset += length_first;
      uint32_t length_second;
      arrToVar(length_second, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_second; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_second-1]=0;
      this->second = (char *)(inbuffer + offset-1);
      offset += length_second;
     return offset;
    }

    virtual const char * getType() override { return "mongodb_store_msgs/StringPair"; };
    virtual const char * getMD5() override { return "c0d0db6e21f3fc1eb068f9cc22ba8beb"; };

  };

}
#endif
