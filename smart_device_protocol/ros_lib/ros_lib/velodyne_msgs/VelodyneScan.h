#ifndef _ROS_velodyne_msgs_VelodyneScan_h
#define _ROS_velodyne_msgs_VelodyneScan_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "velodyne_msgs/VelodynePacket.h"

namespace velodyne_msgs
{

  class VelodyneScan : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      uint32_t packets_length;
      typedef velodyne_msgs::VelodynePacket _packets_type;
      _packets_type st_packets;
      _packets_type * packets;

    VelodyneScan():
      header(),
      packets_length(0), st_packets(), packets(nullptr)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      *(outbuffer + offset + 0) = (this->packets_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->packets_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->packets_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->packets_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->packets_length);
      for( uint32_t i = 0; i < packets_length; i++){
      offset += this->packets[i].serialize(outbuffer + offset);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      uint32_t packets_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      packets_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      packets_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      packets_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->packets_length);
      if(packets_lengthT > packets_length)
        this->packets = (velodyne_msgs::VelodynePacket*)realloc(this->packets, packets_lengthT * sizeof(velodyne_msgs::VelodynePacket));
      packets_length = packets_lengthT;
      for( uint32_t i = 0; i < packets_length; i++){
      offset += this->st_packets.deserialize(inbuffer + offset);
        memcpy( &(this->packets[i]), &(this->st_packets), sizeof(velodyne_msgs::VelodynePacket));
      }
     return offset;
    }

    virtual const char * getType() override { return "velodyne_msgs/VelodyneScan"; };
    virtual const char * getMD5() override { return "50804fc9533a0e579e6322c04ae70566"; };

  };

}
#endif
