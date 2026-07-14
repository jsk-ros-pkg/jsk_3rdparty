#ifndef _ROS_nmea_msgs_Gpgsv_h
#define _ROS_nmea_msgs_Gpgsv_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "nmea_msgs/GpgsvSatellite.h"

namespace nmea_msgs
{

  class Gpgsv : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef const char* _message_id_type;
      _message_id_type message_id;
      typedef uint8_t _n_msgs_type;
      _n_msgs_type n_msgs;
      typedef uint8_t _msg_number_type;
      _msg_number_type msg_number;
      typedef uint8_t _n_satellites_type;
      _n_satellites_type n_satellites;
      uint32_t satellites_length;
      typedef nmea_msgs::GpgsvSatellite _satellites_type;
      _satellites_type st_satellites;
      _satellites_type * satellites;

    Gpgsv():
      header(),
      message_id(""),
      n_msgs(0),
      msg_number(0),
      n_satellites(0),
      satellites_length(0), st_satellites(), satellites(nullptr)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      uint32_t length_message_id = strlen(this->message_id);
      varToArr(outbuffer + offset, length_message_id);
      offset += 4;
      memcpy(outbuffer + offset, this->message_id, length_message_id);
      offset += length_message_id;
      *(outbuffer + offset + 0) = (this->n_msgs >> (8 * 0)) & 0xFF;
      offset += sizeof(this->n_msgs);
      *(outbuffer + offset + 0) = (this->msg_number >> (8 * 0)) & 0xFF;
      offset += sizeof(this->msg_number);
      *(outbuffer + offset + 0) = (this->n_satellites >> (8 * 0)) & 0xFF;
      offset += sizeof(this->n_satellites);
      *(outbuffer + offset + 0) = (this->satellites_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->satellites_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->satellites_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->satellites_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->satellites_length);
      for( uint32_t i = 0; i < satellites_length; i++){
      offset += this->satellites[i].serialize(outbuffer + offset);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      uint32_t length_message_id;
      arrToVar(length_message_id, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_message_id; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_message_id-1]=0;
      this->message_id = (char *)(inbuffer + offset-1);
      offset += length_message_id;
      this->n_msgs =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->n_msgs);
      this->msg_number =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->msg_number);
      this->n_satellites =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->n_satellites);
      uint32_t satellites_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      satellites_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      satellites_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      satellites_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->satellites_length);
      if(satellites_lengthT > satellites_length)
        this->satellites = (nmea_msgs::GpgsvSatellite*)realloc(this->satellites, satellites_lengthT * sizeof(nmea_msgs::GpgsvSatellite));
      satellites_length = satellites_lengthT;
      for( uint32_t i = 0; i < satellites_length; i++){
      offset += this->st_satellites.deserialize(inbuffer + offset);
        memcpy( &(this->satellites[i]), &(this->st_satellites), sizeof(nmea_msgs::GpgsvSatellite));
      }
     return offset;
    }

    virtual const char * getType() override { return "nmea_msgs/Gpgsv"; };
    virtual const char * getMD5() override { return "6f34bebc32fe085313c942a96fd39c77"; };

  };

}
#endif
