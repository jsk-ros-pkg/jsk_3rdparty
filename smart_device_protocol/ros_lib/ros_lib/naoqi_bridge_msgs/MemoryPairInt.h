#ifndef _ROS_naoqi_bridge_msgs_MemoryPairInt_h
#define _ROS_naoqi_bridge_msgs_MemoryPairInt_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace naoqi_bridge_msgs
{

  class MemoryPairInt : public ros::Msg
  {
    public:
      typedef const char* _memoryKey_type;
      _memoryKey_type memoryKey;
      typedef int32_t _data_type;
      _data_type data;

    MemoryPairInt():
      memoryKey(""),
      data(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      uint32_t length_memoryKey = strlen(this->memoryKey);
      varToArr(outbuffer + offset, length_memoryKey);
      offset += 4;
      memcpy(outbuffer + offset, this->memoryKey, length_memoryKey);
      offset += length_memoryKey;
      union {
        int32_t real;
        uint32_t base;
      } u_data;
      u_data.real = this->data;
      *(outbuffer + offset + 0) = (u_data.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_data.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_data.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_data.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->data);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      uint32_t length_memoryKey;
      arrToVar(length_memoryKey, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_memoryKey; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_memoryKey-1]=0;
      this->memoryKey = (char *)(inbuffer + offset-1);
      offset += length_memoryKey;
      union {
        int32_t real;
        uint32_t base;
      } u_data;
      u_data.base = 0;
      u_data.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_data.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_data.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_data.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->data = u_data.real;
      offset += sizeof(this->data);
     return offset;
    }

    virtual const char * getType() override { return "naoqi_bridge_msgs/MemoryPairInt"; };
    virtual const char * getMD5() override { return "22045fae148fc28af04280556676c5b8"; };

  };

}
#endif
