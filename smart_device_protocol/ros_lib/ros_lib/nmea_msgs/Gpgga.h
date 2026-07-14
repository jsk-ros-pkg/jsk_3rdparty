#ifndef _ROS_nmea_msgs_Gpgga_h
#define _ROS_nmea_msgs_Gpgga_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"

namespace nmea_msgs
{

  class Gpgga : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef const char* _message_id_type;
      _message_id_type message_id;
      typedef float _utc_seconds_type;
      _utc_seconds_type utc_seconds;
      typedef float _lat_type;
      _lat_type lat;
      typedef float _lon_type;
      _lon_type lon;
      typedef const char* _lat_dir_type;
      _lat_dir_type lat_dir;
      typedef const char* _lon_dir_type;
      _lon_dir_type lon_dir;
      typedef uint32_t _gps_qual_type;
      _gps_qual_type gps_qual;
      typedef uint32_t _num_sats_type;
      _num_sats_type num_sats;
      typedef float _hdop_type;
      _hdop_type hdop;
      typedef float _alt_type;
      _alt_type alt;
      typedef const char* _altitude_units_type;
      _altitude_units_type altitude_units;
      typedef float _undulation_type;
      _undulation_type undulation;
      typedef const char* _undulation_units_type;
      _undulation_units_type undulation_units;
      typedef uint32_t _diff_age_type;
      _diff_age_type diff_age;
      typedef const char* _station_id_type;
      _station_id_type station_id;

    Gpgga():
      header(),
      message_id(""),
      utc_seconds(0),
      lat(0),
      lon(0),
      lat_dir(""),
      lon_dir(""),
      gps_qual(0),
      num_sats(0),
      hdop(0),
      alt(0),
      altitude_units(""),
      undulation(0),
      undulation_units(""),
      diff_age(0),
      station_id("")
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
      offset += serializeAvrFloat64(outbuffer + offset, this->utc_seconds);
      offset += serializeAvrFloat64(outbuffer + offset, this->lat);
      offset += serializeAvrFloat64(outbuffer + offset, this->lon);
      uint32_t length_lat_dir = strlen(this->lat_dir);
      varToArr(outbuffer + offset, length_lat_dir);
      offset += 4;
      memcpy(outbuffer + offset, this->lat_dir, length_lat_dir);
      offset += length_lat_dir;
      uint32_t length_lon_dir = strlen(this->lon_dir);
      varToArr(outbuffer + offset, length_lon_dir);
      offset += 4;
      memcpy(outbuffer + offset, this->lon_dir, length_lon_dir);
      offset += length_lon_dir;
      *(outbuffer + offset + 0) = (this->gps_qual >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->gps_qual >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->gps_qual >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->gps_qual >> (8 * 3)) & 0xFF;
      offset += sizeof(this->gps_qual);
      *(outbuffer + offset + 0) = (this->num_sats >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->num_sats >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->num_sats >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->num_sats >> (8 * 3)) & 0xFF;
      offset += sizeof(this->num_sats);
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
      } u_alt;
      u_alt.real = this->alt;
      *(outbuffer + offset + 0) = (u_alt.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_alt.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_alt.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_alt.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->alt);
      uint32_t length_altitude_units = strlen(this->altitude_units);
      varToArr(outbuffer + offset, length_altitude_units);
      offset += 4;
      memcpy(outbuffer + offset, this->altitude_units, length_altitude_units);
      offset += length_altitude_units;
      union {
        float real;
        uint32_t base;
      } u_undulation;
      u_undulation.real = this->undulation;
      *(outbuffer + offset + 0) = (u_undulation.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_undulation.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_undulation.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_undulation.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->undulation);
      uint32_t length_undulation_units = strlen(this->undulation_units);
      varToArr(outbuffer + offset, length_undulation_units);
      offset += 4;
      memcpy(outbuffer + offset, this->undulation_units, length_undulation_units);
      offset += length_undulation_units;
      *(outbuffer + offset + 0) = (this->diff_age >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->diff_age >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->diff_age >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->diff_age >> (8 * 3)) & 0xFF;
      offset += sizeof(this->diff_age);
      uint32_t length_station_id = strlen(this->station_id);
      varToArr(outbuffer + offset, length_station_id);
      offset += 4;
      memcpy(outbuffer + offset, this->station_id, length_station_id);
      offset += length_station_id;
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
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->utc_seconds));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->lat));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->lon));
      uint32_t length_lat_dir;
      arrToVar(length_lat_dir, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_lat_dir; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_lat_dir-1]=0;
      this->lat_dir = (char *)(inbuffer + offset-1);
      offset += length_lat_dir;
      uint32_t length_lon_dir;
      arrToVar(length_lon_dir, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_lon_dir; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_lon_dir-1]=0;
      this->lon_dir = (char *)(inbuffer + offset-1);
      offset += length_lon_dir;
      this->gps_qual =  ((uint32_t) (*(inbuffer + offset)));
      this->gps_qual |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->gps_qual |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->gps_qual |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->gps_qual);
      this->num_sats =  ((uint32_t) (*(inbuffer + offset)));
      this->num_sats |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->num_sats |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->num_sats |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->num_sats);
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
      } u_alt;
      u_alt.base = 0;
      u_alt.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_alt.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_alt.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_alt.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->alt = u_alt.real;
      offset += sizeof(this->alt);
      uint32_t length_altitude_units;
      arrToVar(length_altitude_units, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_altitude_units; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_altitude_units-1]=0;
      this->altitude_units = (char *)(inbuffer + offset-1);
      offset += length_altitude_units;
      union {
        float real;
        uint32_t base;
      } u_undulation;
      u_undulation.base = 0;
      u_undulation.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_undulation.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_undulation.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_undulation.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->undulation = u_undulation.real;
      offset += sizeof(this->undulation);
      uint32_t length_undulation_units;
      arrToVar(length_undulation_units, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_undulation_units; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_undulation_units-1]=0;
      this->undulation_units = (char *)(inbuffer + offset-1);
      offset += length_undulation_units;
      this->diff_age =  ((uint32_t) (*(inbuffer + offset)));
      this->diff_age |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->diff_age |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->diff_age |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->diff_age);
      uint32_t length_station_id;
      arrToVar(length_station_id, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_station_id; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_station_id-1]=0;
      this->station_id = (char *)(inbuffer + offset-1);
      offset += length_station_id;
     return offset;
    }

    virtual const char * getType() override { return "nmea_msgs/Gpgga"; };
    virtual const char * getMD5() override { return "8f51cb504898f39d8ad9f698f0b6f9cd"; };

  };

}
#endif
