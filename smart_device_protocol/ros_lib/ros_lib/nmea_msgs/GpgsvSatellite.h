#ifndef _ROS_nmea_msgs_GpgsvSatellite_h
#define _ROS_nmea_msgs_GpgsvSatellite_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace nmea_msgs
{

  class GpgsvSatellite : public ros::Msg
  {
    public:
      typedef uint8_t _prn_type;
      _prn_type prn;
      typedef uint8_t _elevation_type;
      _elevation_type elevation;
      typedef uint16_t _azimuth_type;
      _azimuth_type azimuth;
      typedef int8_t _snr_type;
      _snr_type snr;

    GpgsvSatellite():
      prn(0),
      elevation(0),
      azimuth(0),
      snr(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->prn >> (8 * 0)) & 0xFF;
      offset += sizeof(this->prn);
      *(outbuffer + offset + 0) = (this->elevation >> (8 * 0)) & 0xFF;
      offset += sizeof(this->elevation);
      *(outbuffer + offset + 0) = (this->azimuth >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->azimuth >> (8 * 1)) & 0xFF;
      offset += sizeof(this->azimuth);
      union {
        int8_t real;
        uint8_t base;
      } u_snr;
      u_snr.real = this->snr;
      *(outbuffer + offset + 0) = (u_snr.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->snr);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      this->prn =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->prn);
      this->elevation =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->elevation);
      this->azimuth =  ((uint16_t) (*(inbuffer + offset)));
      this->azimuth |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->azimuth);
      union {
        int8_t real;
        uint8_t base;
      } u_snr;
      u_snr.base = 0;
      u_snr.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->snr = u_snr.real;
      offset += sizeof(this->snr);
     return offset;
    }

    virtual const char * getType() override { return "nmea_msgs/GpgsvSatellite"; };
    virtual const char * getMD5() override { return "d862f2ce05a26a83264a8add99c7b668"; };

  };

}
#endif
