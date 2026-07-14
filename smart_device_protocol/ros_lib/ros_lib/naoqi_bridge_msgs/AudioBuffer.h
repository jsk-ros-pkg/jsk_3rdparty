#ifndef _ROS_naoqi_bridge_msgs_AudioBuffer_h
#define _ROS_naoqi_bridge_msgs_AudioBuffer_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"

namespace naoqi_bridge_msgs
{

  class AudioBuffer : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef uint16_t _frequency_type;
      _frequency_type frequency;
      uint32_t channelMap_length;
      typedef uint8_t _channelMap_type;
      _channelMap_type st_channelMap;
      _channelMap_type * channelMap;
      uint32_t data_length;
      typedef int16_t _data_type;
      _data_type st_data;
      _data_type * data;
      enum { CHANNEL_FRONT_LEFT = 0 };
      enum { CHANNEL_FRONT_CENTER = 1 };
      enum { CHANNEL_FRONT_RIGHT = 2 };
      enum { CHANNEL_REAR_LEFT = 3 };
      enum { CHANNEL_REAR_CENTER = 4 };
      enum { CHANNEL_REAR_RIGHT = 5 };
      enum { CHANNEL_SURROUND_LEFT = 6 };
      enum { CHANNEL_SURROUND_RIGHT = 7 };
      enum { CHANNEL_SUBWOOFER = 8 };
      enum { CHANNEL_LFE = 9 };

    AudioBuffer():
      header(),
      frequency(0),
      channelMap_length(0), st_channelMap(), channelMap(nullptr),
      data_length(0), st_data(), data(nullptr)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      *(outbuffer + offset + 0) = (this->frequency >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->frequency >> (8 * 1)) & 0xFF;
      offset += sizeof(this->frequency);
      *(outbuffer + offset + 0) = (this->channelMap_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->channelMap_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->channelMap_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->channelMap_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->channelMap_length);
      for( uint32_t i = 0; i < channelMap_length; i++){
      *(outbuffer + offset + 0) = (this->channelMap[i] >> (8 * 0)) & 0xFF;
      offset += sizeof(this->channelMap[i]);
      }
      *(outbuffer + offset + 0) = (this->data_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->data_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->data_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->data_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->data_length);
      for( uint32_t i = 0; i < data_length; i++){
      union {
        int16_t real;
        uint16_t base;
      } u_datai;
      u_datai.real = this->data[i];
      *(outbuffer + offset + 0) = (u_datai.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_datai.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->data[i]);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      this->frequency =  ((uint16_t) (*(inbuffer + offset)));
      this->frequency |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->frequency);
      uint32_t channelMap_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      channelMap_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      channelMap_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      channelMap_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->channelMap_length);
      if(channelMap_lengthT > channelMap_length)
        this->channelMap = (uint8_t*)realloc(this->channelMap, channelMap_lengthT * sizeof(uint8_t));
      channelMap_length = channelMap_lengthT;
      for( uint32_t i = 0; i < channelMap_length; i++){
      this->st_channelMap =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->st_channelMap);
        memcpy( &(this->channelMap[i]), &(this->st_channelMap), sizeof(uint8_t));
      }
      uint32_t data_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      data_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      data_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      data_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->data_length);
      if(data_lengthT > data_length)
        this->data = (int16_t*)realloc(this->data, data_lengthT * sizeof(int16_t));
      data_length = data_lengthT;
      for( uint32_t i = 0; i < data_length; i++){
      union {
        int16_t real;
        uint16_t base;
      } u_st_data;
      u_st_data.base = 0;
      u_st_data.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_data.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->st_data = u_st_data.real;
      offset += sizeof(this->st_data);
        memcpy( &(this->data[i]), &(this->st_data), sizeof(int16_t));
      }
     return offset;
    }

    virtual const char * getType() override { return "naoqi_bridge_msgs/AudioBuffer"; };
    virtual const char * getMD5() override { return "50f300aa63f3c1b2f3d3173329165316"; };

  };

}
#endif
