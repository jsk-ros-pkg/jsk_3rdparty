#ifndef _ROS_SERVICE_BoardConfig_h
#define _ROS_SERVICE_BoardConfig_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace wge100_camera
{

static const char BOARDCONFIG[] = "wge100_camera/BoardConfig";

  class BoardConfigRequest : public ros::Msg
  {
    public:
      typedef uint32_t _serial_type;
      _serial_type serial;
      uint32_t mac_length;
      typedef uint8_t _mac_type;
      _mac_type st_mac;
      _mac_type * mac;

    BoardConfigRequest():
      serial(0),
      mac_length(0), st_mac(), mac(nullptr)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->serial >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->serial >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->serial >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->serial >> (8 * 3)) & 0xFF;
      offset += sizeof(this->serial);
      *(outbuffer + offset + 0) = (this->mac_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->mac_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->mac_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->mac_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->mac_length);
      for( uint32_t i = 0; i < mac_length; i++){
      *(outbuffer + offset + 0) = (this->mac[i] >> (8 * 0)) & 0xFF;
      offset += sizeof(this->mac[i]);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      this->serial =  ((uint32_t) (*(inbuffer + offset)));
      this->serial |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->serial |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->serial |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->serial);
      uint32_t mac_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      mac_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      mac_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      mac_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->mac_length);
      if(mac_lengthT > mac_length)
        this->mac = (uint8_t*)realloc(this->mac, mac_lengthT * sizeof(uint8_t));
      mac_length = mac_lengthT;
      for( uint32_t i = 0; i < mac_length; i++){
      this->st_mac =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->st_mac);
        memcpy( &(this->mac[i]), &(this->st_mac), sizeof(uint8_t));
      }
     return offset;
    }

    virtual const char * getType() override { return BOARDCONFIG; };
    virtual const char * getMD5() override { return "ec9bad54b410ebc79183d761c609dd76"; };

  };

  class BoardConfigResponse : public ros::Msg
  {
    public:
      typedef uint8_t _success_type;
      _success_type success;

    BoardConfigResponse():
      success(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->success >> (8 * 0)) & 0xFF;
      offset += sizeof(this->success);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      this->success =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->success);
     return offset;
    }

    virtual const char * getType() override { return BOARDCONFIG; };
    virtual const char * getMD5() override { return "6cca7c80398b8b31af04b80534923f16"; };

  };

  class BoardConfig {
    public:
    typedef BoardConfigRequest Request;
    typedef BoardConfigResponse Response;
  };

}
#endif
