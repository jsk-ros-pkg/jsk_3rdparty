#ifndef _ROS_fetch_driver_msgs_DisableChargingGoal_h
#define _ROS_fetch_driver_msgs_DisableChargingGoal_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "ros/duration.h"

namespace fetch_driver_msgs
{

  class DisableChargingGoal : public ros::Msg
  {
    public:
      typedef ros::Duration _disable_duration_type;
      _disable_duration_type disable_duration;

    DisableChargingGoal():
      disable_duration()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->disable_duration.sec >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->disable_duration.sec >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->disable_duration.sec >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->disable_duration.sec >> (8 * 3)) & 0xFF;
      offset += sizeof(this->disable_duration.sec);
      *(outbuffer + offset + 0) = (this->disable_duration.nsec >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->disable_duration.nsec >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->disable_duration.nsec >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->disable_duration.nsec >> (8 * 3)) & 0xFF;
      offset += sizeof(this->disable_duration.nsec);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      this->disable_duration.sec =  ((uint32_t) (*(inbuffer + offset)));
      this->disable_duration.sec |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->disable_duration.sec |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->disable_duration.sec |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->disable_duration.sec);
      this->disable_duration.nsec =  ((uint32_t) (*(inbuffer + offset)));
      this->disable_duration.nsec |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->disable_duration.nsec |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->disable_duration.nsec |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->disable_duration.nsec);
     return offset;
    }

    virtual const char * getType() override { return "fetch_driver_msgs/DisableChargingGoal"; };
    virtual const char * getMD5() override { return "53aa7041ef836ed316df1a458b2d4178"; };

  };

}
#endif
