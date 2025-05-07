#ifndef _ROS_rosbridge_msgs_ConnectedClient_h
#define _ROS_rosbridge_msgs_ConnectedClient_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "ros/time.h"

namespace rosbridge_msgs
{

  class ConnectedClient : public ros::Msg
  {
    public:
      typedef const char* _ip_address_type;
      _ip_address_type ip_address;
      typedef ros::Time _connection_time_type;
      _connection_time_type connection_time;

    ConnectedClient():
      ip_address(""),
      connection_time()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      uint32_t length_ip_address = strlen(this->ip_address);
      varToArr(outbuffer + offset, length_ip_address);
      offset += 4;
      memcpy(outbuffer + offset, this->ip_address, length_ip_address);
      offset += length_ip_address;
      *(outbuffer + offset + 0) = (this->connection_time.sec >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->connection_time.sec >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->connection_time.sec >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->connection_time.sec >> (8 * 3)) & 0xFF;
      offset += sizeof(this->connection_time.sec);
      *(outbuffer + offset + 0) = (this->connection_time.nsec >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->connection_time.nsec >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->connection_time.nsec >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->connection_time.nsec >> (8 * 3)) & 0xFF;
      offset += sizeof(this->connection_time.nsec);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      uint32_t length_ip_address;
      arrToVar(length_ip_address, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_ip_address; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_ip_address-1]=0;
      this->ip_address = (char *)(inbuffer + offset-1);
      offset += length_ip_address;
      this->connection_time.sec =  ((uint32_t) (*(inbuffer + offset)));
      this->connection_time.sec |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->connection_time.sec |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->connection_time.sec |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->connection_time.sec);
      this->connection_time.nsec =  ((uint32_t) (*(inbuffer + offset)));
      this->connection_time.nsec |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->connection_time.nsec |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->connection_time.nsec |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->connection_time.nsec);
     return offset;
    }

    virtual const char * getType() override { return "rosbridge_msgs/ConnectedClient"; };
    virtual const char * getMD5() override { return "7f2187ce389b39b2b3bb2a3957e54c04"; };

  };

}
#endif
