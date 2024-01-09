#ifndef _ROS_naoqi_bridge_msgs_StringArrayStamped_h
#define _ROS_naoqi_bridge_msgs_StringArrayStamped_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"

namespace naoqi_bridge_msgs
{

  class StringArrayStamped : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      uint32_t data_length;
      typedef char* _data_type;
      _data_type st_data;
      _data_type * data;

    StringArrayStamped():
      header(),
      data_length(0), st_data(), data(nullptr)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
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
      offset += this->header.deserialize(inbuffer + offset);
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

    virtual const char * getType() override { return "naoqi_bridge_msgs/StringArrayStamped"; };
    virtual const char * getMD5() override { return "17b6e4aa81015d95bcd2b08039bd6bda"; };

  };

}
#endif
