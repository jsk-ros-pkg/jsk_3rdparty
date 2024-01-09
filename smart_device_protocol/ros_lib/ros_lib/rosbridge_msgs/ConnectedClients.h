#ifndef _ROS_rosbridge_msgs_ConnectedClients_h
#define _ROS_rosbridge_msgs_ConnectedClients_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "rosbridge_msgs/ConnectedClient.h"

namespace rosbridge_msgs
{

  class ConnectedClients : public ros::Msg
  {
    public:
      uint32_t clients_length;
      typedef rosbridge_msgs::ConnectedClient _clients_type;
      _clients_type st_clients;
      _clients_type * clients;

    ConnectedClients():
      clients_length(0), st_clients(), clients(nullptr)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->clients_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->clients_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->clients_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->clients_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->clients_length);
      for( uint32_t i = 0; i < clients_length; i++){
      offset += this->clients[i].serialize(outbuffer + offset);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      uint32_t clients_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      clients_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      clients_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      clients_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->clients_length);
      if(clients_lengthT > clients_length)
        this->clients = (rosbridge_msgs::ConnectedClient*)realloc(this->clients, clients_lengthT * sizeof(rosbridge_msgs::ConnectedClient));
      clients_length = clients_lengthT;
      for( uint32_t i = 0; i < clients_length; i++){
      offset += this->st_clients.deserialize(inbuffer + offset);
        memcpy( &(this->clients[i]), &(this->st_clients), sizeof(rosbridge_msgs::ConnectedClient));
      }
     return offset;
    }

    virtual const char * getType() override { return "rosbridge_msgs/ConnectedClients"; };
    virtual const char * getMD5() override { return "d0d53b0c0aa23aa7e4cf52f49bca4b69"; };

  };

}
#endif
