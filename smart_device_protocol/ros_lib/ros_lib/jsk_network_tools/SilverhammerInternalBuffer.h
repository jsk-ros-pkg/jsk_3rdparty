#ifndef _ROS_jsk_network_tools_SilverhammerInternalBuffer_h
#define _ROS_jsk_network_tools_SilverhammerInternalBuffer_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"

namespace jsk_network_tools
{

  class SilverhammerInternalBuffer : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef uint32_t _seq_id_type;
      _seq_id_type seq_id;
      uint32_t data_length;
      typedef uint8_t _data_type;
      _data_type st_data;
      _data_type * data;

    SilverhammerInternalBuffer():
      header(),
      seq_id(0),
      data_length(0), st_data(), data(nullptr)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      *(outbuffer + offset + 0) = (this->seq_id >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->seq_id >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->seq_id >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->seq_id >> (8 * 3)) & 0xFF;
      offset += sizeof(this->seq_id);
      *(outbuffer + offset + 0) = (this->data_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->data_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->data_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->data_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->data_length);
      for( uint32_t i = 0; i < data_length; i++){
      *(outbuffer + offset + 0) = (this->data[i] >> (8 * 0)) & 0xFF;
      offset += sizeof(this->data[i]);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      this->seq_id =  ((uint32_t) (*(inbuffer + offset)));
      this->seq_id |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->seq_id |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->seq_id |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->seq_id);
      uint32_t data_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      data_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      data_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      data_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->data_length);
      if(data_lengthT > data_length)
        this->data = (uint8_t*)realloc(this->data, data_lengthT * sizeof(uint8_t));
      data_length = data_lengthT;
      for( uint32_t i = 0; i < data_length; i++){
      this->st_data =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->st_data);
        memcpy( &(this->data[i]), &(this->st_data), sizeof(uint8_t));
      }
     return offset;
    }

    virtual const char * getType() override { return "jsk_network_tools/SilverhammerInternalBuffer"; };
    virtual const char * getMD5() override { return "b0224c297e0e2b6e1ac36f4f9188136f"; };

  };

}
#endif
