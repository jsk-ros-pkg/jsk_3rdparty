#ifndef _ROS_naoqi_bridge_msgs_MemoryList_h
#define _ROS_naoqi_bridge_msgs_MemoryList_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "naoqi_bridge_msgs/MemoryPairString.h"
#include "naoqi_bridge_msgs/MemoryPairInt.h"
#include "naoqi_bridge_msgs/MemoryPairFloat.h"

namespace naoqi_bridge_msgs
{

  class MemoryList : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      uint32_t strings_length;
      typedef naoqi_bridge_msgs::MemoryPairString _strings_type;
      _strings_type st_strings;
      _strings_type * strings;
      uint32_t ints_length;
      typedef naoqi_bridge_msgs::MemoryPairInt _ints_type;
      _ints_type st_ints;
      _ints_type * ints;
      uint32_t floats_length;
      typedef naoqi_bridge_msgs::MemoryPairFloat _floats_type;
      _floats_type st_floats;
      _floats_type * floats;

    MemoryList():
      header(),
      strings_length(0), st_strings(), strings(nullptr),
      ints_length(0), st_ints(), ints(nullptr),
      floats_length(0), st_floats(), floats(nullptr)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      *(outbuffer + offset + 0) = (this->strings_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->strings_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->strings_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->strings_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->strings_length);
      for( uint32_t i = 0; i < strings_length; i++){
      offset += this->strings[i].serialize(outbuffer + offset);
      }
      *(outbuffer + offset + 0) = (this->ints_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->ints_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->ints_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->ints_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->ints_length);
      for( uint32_t i = 0; i < ints_length; i++){
      offset += this->ints[i].serialize(outbuffer + offset);
      }
      *(outbuffer + offset + 0) = (this->floats_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->floats_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->floats_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->floats_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->floats_length);
      for( uint32_t i = 0; i < floats_length; i++){
      offset += this->floats[i].serialize(outbuffer + offset);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      uint32_t strings_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      strings_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      strings_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      strings_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->strings_length);
      if(strings_lengthT > strings_length)
        this->strings = (naoqi_bridge_msgs::MemoryPairString*)realloc(this->strings, strings_lengthT * sizeof(naoqi_bridge_msgs::MemoryPairString));
      strings_length = strings_lengthT;
      for( uint32_t i = 0; i < strings_length; i++){
      offset += this->st_strings.deserialize(inbuffer + offset);
        memcpy( &(this->strings[i]), &(this->st_strings), sizeof(naoqi_bridge_msgs::MemoryPairString));
      }
      uint32_t ints_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      ints_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      ints_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      ints_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->ints_length);
      if(ints_lengthT > ints_length)
        this->ints = (naoqi_bridge_msgs::MemoryPairInt*)realloc(this->ints, ints_lengthT * sizeof(naoqi_bridge_msgs::MemoryPairInt));
      ints_length = ints_lengthT;
      for( uint32_t i = 0; i < ints_length; i++){
      offset += this->st_ints.deserialize(inbuffer + offset);
        memcpy( &(this->ints[i]), &(this->st_ints), sizeof(naoqi_bridge_msgs::MemoryPairInt));
      }
      uint32_t floats_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      floats_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      floats_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      floats_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->floats_length);
      if(floats_lengthT > floats_length)
        this->floats = (naoqi_bridge_msgs::MemoryPairFloat*)realloc(this->floats, floats_lengthT * sizeof(naoqi_bridge_msgs::MemoryPairFloat));
      floats_length = floats_lengthT;
      for( uint32_t i = 0; i < floats_length; i++){
      offset += this->st_floats.deserialize(inbuffer + offset);
        memcpy( &(this->floats[i]), &(this->st_floats), sizeof(naoqi_bridge_msgs::MemoryPairFloat));
      }
     return offset;
    }

    virtual const char * getType() override { return "naoqi_bridge_msgs/MemoryList"; };
    virtual const char * getMD5() override { return "7222936d1c205b51fbfdb13e468998ad"; };

  };

}
#endif
