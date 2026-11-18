#ifndef _ROS_smart_device_protocol_Packet_h
#define _ROS_smart_device_protocol_Packet_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace smart_device_protocol
{

  class Packet : public ros::Msg
  {
    public:
      uint32_t mac_address_length;
      typedef uint8_t _mac_address_type;
      _mac_address_type st_mac_address;
      _mac_address_type * mac_address;
      uint32_t data_length;
      typedef uint8_t _data_type;
      _data_type st_data;
      _data_type * data;
      enum { PACKET_TYPE_NONE =  0 };
      enum { PACKET_TYPE_TEST =  1 };
      enum { PACKET_TYPE_NAMED_STRING =  11 };
      enum { PACKET_TYPE_NAMED_INT =  12 };
      enum { PACKET_TYPE_NAMED_FLOAT =  13 };
      enum { PACKET_TYPE_SENSOR_ENV_III =  21 };
      enum { PACKET_TYPE_SENSOR_UNITV2_PERSON_COUNTER =  22 };
      enum { PACKET_TYPE_EMERGENCY =  31 };
      enum { PACKET_TYPE_TASK_DISPATCHER =  32 };
      enum { PACKET_TYPE_TASK_RESULT =  33 };
      enum { PACKET_TYPE_TASK_RECEIVED =  34 };
      enum { PACKET_TYPE_DEVICE_MESSAGE_BOARD_META =  41 };
      enum { PACKET_TYPE_DEVICE_MESSAGE_BOARD_DATA =  42 };
      enum { PACKET_TYPE_META =  81 };
      enum { PACKET_TYPE_DATA =  82 };

    Packet():
      mac_address_length(0), st_mac_address(), mac_address(nullptr),
      data_length(0), st_data(), data(nullptr)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->mac_address_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->mac_address_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->mac_address_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->mac_address_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->mac_address_length);
      for( uint32_t i = 0; i < mac_address_length; i++){
      *(outbuffer + offset + 0) = (this->mac_address[i] >> (8 * 0)) & 0xFF;
      offset += sizeof(this->mac_address[i]);
      }
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
      uint32_t mac_address_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      mac_address_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      mac_address_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      mac_address_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->mac_address_length);
      if(mac_address_lengthT > mac_address_length)
        this->mac_address = (uint8_t*)realloc(this->mac_address, mac_address_lengthT * sizeof(uint8_t));
      mac_address_length = mac_address_lengthT;
      for( uint32_t i = 0; i < mac_address_length; i++){
      this->st_mac_address =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->st_mac_address);
        memcpy( &(this->mac_address[i]), &(this->st_mac_address), sizeof(uint8_t));
      }
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

    virtual const char * getType() override { return "smart_device_protocol/Packet"; };
    virtual const char * getMD5() override { return "dbab45830b3b1d11bc00c2acc0192a63"; };

  };

}
#endif
