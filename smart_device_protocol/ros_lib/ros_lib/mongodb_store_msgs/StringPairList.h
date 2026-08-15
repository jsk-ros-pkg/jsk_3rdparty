#ifndef _ROS_mongodb_store_msgs_StringPairList_h
#define _ROS_mongodb_store_msgs_StringPairList_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "mongodb_store_msgs/StringPair.h"

namespace mongodb_store_msgs
{

  class StringPairList : public ros::Msg
  {
    public:
      uint32_t pairs_length;
      typedef mongodb_store_msgs::StringPair _pairs_type;
      _pairs_type st_pairs;
      _pairs_type * pairs;

    StringPairList():
      pairs_length(0), st_pairs(), pairs(nullptr)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->pairs_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->pairs_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->pairs_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->pairs_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->pairs_length);
      for( uint32_t i = 0; i < pairs_length; i++){
      offset += this->pairs[i].serialize(outbuffer + offset);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      uint32_t pairs_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      pairs_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      pairs_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      pairs_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->pairs_length);
      if(pairs_lengthT > pairs_length)
        this->pairs = (mongodb_store_msgs::StringPair*)realloc(this->pairs, pairs_lengthT * sizeof(mongodb_store_msgs::StringPair));
      pairs_length = pairs_lengthT;
      for( uint32_t i = 0; i < pairs_length; i++){
      offset += this->st_pairs.deserialize(inbuffer + offset);
        memcpy( &(this->pairs[i]), &(this->st_pairs), sizeof(mongodb_store_msgs::StringPair));
      }
     return offset;
    }

    virtual const char * getType() override { return "mongodb_store_msgs/StringPairList"; };
    virtual const char * getMD5() override { return "50c368c0f345d8de86876a3bada40aad"; };

  };

}
#endif
