#ifndef _ROS_SERVICE_RequireMonitorStatus_h
#define _ROS_SERVICE_RequireMonitorStatus_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"

namespace jsk_footstep_controller
{

static const char REQUIREMONITORSTATUS[] = "jsk_footstep_controller/RequireMonitorStatus";

  class RequireMonitorStatusRequest : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef float _threshold_type;
      _threshold_type threshold;

    RequireMonitorStatusRequest():
      header(),
      threshold(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      union {
        float real;
        uint32_t base;
      } u_threshold;
      u_threshold.real = this->threshold;
      *(outbuffer + offset + 0) = (u_threshold.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_threshold.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_threshold.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_threshold.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->threshold);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      union {
        float real;
        uint32_t base;
      } u_threshold;
      u_threshold.base = 0;
      u_threshold.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_threshold.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_threshold.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_threshold.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->threshold = u_threshold.real;
      offset += sizeof(this->threshold);
     return offset;
    }

    virtual const char * getType() override { return REQUIREMONITORSTATUS; };
    virtual const char * getMD5() override { return "3dc898e59e7081935abe13d2f48debb8"; };

  };

  class RequireMonitorStatusResponse : public ros::Msg
  {
    public:
      typedef bool _go_type;
      _go_type go;

    RequireMonitorStatusResponse():
      go(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_go;
      u_go.real = this->go;
      *(outbuffer + offset + 0) = (u_go.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->go);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_go;
      u_go.base = 0;
      u_go.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->go = u_go.real;
      offset += sizeof(this->go);
     return offset;
    }

    virtual const char * getType() override { return REQUIREMONITORSTATUS; };
    virtual const char * getMD5() override { return "50769ce61ea599387c084cd1aa050412"; };

  };

  class RequireMonitorStatus {
    public:
    typedef RequireMonitorStatusRequest Request;
    typedef RequireMonitorStatusResponse Response;
  };

}
#endif
