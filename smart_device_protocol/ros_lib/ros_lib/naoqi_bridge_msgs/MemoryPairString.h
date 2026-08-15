#ifndef _ROS_naoqi_bridge_msgs_MemoryPairString_h
#define _ROS_naoqi_bridge_msgs_MemoryPairString_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace naoqi_bridge_msgs
{

  class MemoryPairString : public ros::Msg
  {
    public:
      typedef const char* _memoryKey_type;
      _memoryKey_type memoryKey;
      typedef const char* _data_type;
      _data_type data;

    MemoryPairString():
      memoryKey(""),
      data("")
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
      uint32_t length_data = strlen(this->data);
      varToArr(outbuffer + offset, length_data);
      offset += 4;
      memcpy(outbuffer + offset, this->data, length_data);
      offset += length_data;
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
      uint32_t length_data;
      arrToVar(length_data, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_data; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_data-1]=0;
      this->data = (char *)(inbuffer + offset-1);
      offset += length_data;
     return offset;
    }

    virtual const char * getType() override { return "naoqi_bridge_msgs/MemoryPairString"; };
    virtual const char * getMD5() override { return "b6046f2881035869712dcfeda0628929"; };

  };

}
#endif
