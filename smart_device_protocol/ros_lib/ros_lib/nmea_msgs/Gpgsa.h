#ifndef _ROS_nmea_msgs_Gpgsa_h
#define _ROS_nmea_msgs_Gpgsa_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"

namespace nmea_msgs
{

  class Gpgsa : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef const char* _message_id_type;
      _message_id_type message_id;
      typedef const char* _auto_manual_mode_type;
      _auto_manual_mode_type auto_manual_mode;
      typedef uint8_t _fix_mode_type;
      _fix_mode_type fix_mode;
      uint32_t sv_ids_length;
      typedef uint8_t _sv_ids_type;
      _sv_ids_type st_sv_ids;
      _sv_ids_type * sv_ids;
      typedef float _pdop_type;
      _pdop_type pdop;
      typedef float _hdop_type;
      _hdop_type hdop;
      typedef float _vdop_type;
      _vdop_type vdop;

    Gpgsa():
      header(),
      message_id(""),
      auto_manual_mode(""),
      fix_mode(0),
      sv_ids_length(0), st_sv_ids(), sv_ids(nullptr),
      pdop(0),
      hdop(0),
      vdop(0)
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
      uint32_t length_auto_manual_mode = strlen(this->auto_manual_mode);
      varToArr(outbuffer + offset, length_auto_manual_mode);
      offset += 4;
      memcpy(outbuffer + offset, this->auto_manual_mode, length_auto_manual_mode);
      offset += length_auto_manual_mode;
      *(outbuffer + offset + 0) = (this->fix_mode >> (8 * 0)) & 0xFF;
      offset += sizeof(this->fix_mode);
      *(outbuffer + offset + 0) = (this->sv_ids_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->sv_ids_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->sv_ids_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->sv_ids_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->sv_ids_length);
      for( uint32_t i = 0; i < sv_ids_length; i++){
      *(outbuffer + offset + 0) = (this->sv_ids[i] >> (8 * 0)) & 0xFF;
      offset += sizeof(this->sv_ids[i]);
      }
      union {
        float real;
        uint32_t base;
      } u_pdop;
      u_pdop.real = this->pdop;
      *(outbuffer + offset + 0) = (u_pdop.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_pdop.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_pdop.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_pdop.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->pdop);
      union {
        float real;
        uint32_t base;
      } u_hdop;
      u_hdop.real = this->hdop;
      *(outbuffer + offset + 0) = (u_hdop.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_hdop.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_hdop.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_hdop.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->hdop);
      union {
        float real;
        uint32_t base;
      } u_vdop;
      u_vdop.real = this->vdop;
      *(outbuffer + offset + 0) = (u_vdop.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_vdop.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_vdop.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_vdop.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->vdop);
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
      uint32_t length_auto_manual_mode;
      arrToVar(length_auto_manual_mode, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_auto_manual_mode; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_auto_manual_mode-1]=0;
      this->auto_manual_mode = (char *)(inbuffer + offset-1);
      offset += length_auto_manual_mode;
      this->fix_mode =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->fix_mode);
      uint32_t sv_ids_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      sv_ids_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      sv_ids_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      sv_ids_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->sv_ids_length);
      if(sv_ids_lengthT > sv_ids_length)
        this->sv_ids = (uint8_t*)realloc(this->sv_ids, sv_ids_lengthT * sizeof(uint8_t));
      sv_ids_length = sv_ids_lengthT;
      for( uint32_t i = 0; i < sv_ids_length; i++){
      this->st_sv_ids =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->st_sv_ids);
        memcpy( &(this->sv_ids[i]), &(this->st_sv_ids), sizeof(uint8_t));
      }
      union {
        float real;
        uint32_t base;
      } u_pdop;
      u_pdop.base = 0;
      u_pdop.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_pdop.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_pdop.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_pdop.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->pdop = u_pdop.real;
      offset += sizeof(this->pdop);
      union {
        float real;
        uint32_t base;
      } u_hdop;
      u_hdop.base = 0;
      u_hdop.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_hdop.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_hdop.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_hdop.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->hdop = u_hdop.real;
      offset += sizeof(this->hdop);
      union {
        float real;
        uint32_t base;
      } u_vdop;
      u_vdop.base = 0;
      u_vdop.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_vdop.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_vdop.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_vdop.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->vdop = u_vdop.real;
      offset += sizeof(this->vdop);
     return offset;
    }

    virtual const char * getType() override { return "nmea_msgs/Gpgsa"; };
    virtual const char * getMD5() override { return "94a6ef4a36d322374b16097a5d03433e"; };

  };

}
#endif
