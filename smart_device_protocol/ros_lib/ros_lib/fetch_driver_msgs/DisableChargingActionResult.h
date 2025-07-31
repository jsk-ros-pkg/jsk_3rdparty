#ifndef _ROS_fetch_driver_msgs_DisableChargingActionResult_h
#define _ROS_fetch_driver_msgs_DisableChargingActionResult_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "actionlib_msgs/GoalStatus.h"
#include "fetch_driver_msgs/DisableChargingResult.h"

namespace fetch_driver_msgs
{

  class DisableChargingActionResult : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef actionlib_msgs::GoalStatus _status_type;
      _status_type status;
      typedef fetch_driver_msgs::DisableChargingResult _result_type;
      _result_type result;

    DisableChargingActionResult():
      header(),
      status(),
      result()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      offset += this->status.serialize(outbuffer + offset);
      offset += this->result.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      offset += this->status.deserialize(inbuffer + offset);
      offset += this->result.deserialize(inbuffer + offset);
     return offset;
    }

    virtual const char * getType() override { return "fetch_driver_msgs/DisableChargingActionResult"; };
    virtual const char * getMD5() override { return "303f1e5dc2c6adb3e46cf4339a1cdb1d"; };

  };

}
#endif
