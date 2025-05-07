#ifndef _ROS_franka_gripper_StopActionResult_h
#define _ROS_franka_gripper_StopActionResult_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "actionlib_msgs/GoalStatus.h"
#include "franka_gripper/StopResult.h"

namespace franka_gripper
{

  class StopActionResult : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef actionlib_msgs::GoalStatus _status_type;
      _status_type status;
      typedef franka_gripper::StopResult _result_type;
      _result_type result;

    StopActionResult():
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

    virtual const char * getType() override { return "franka_gripper/StopActionResult"; };
    virtual const char * getMD5() override { return "89dbc4e75593b525bbbea3a150532ed6"; };

  };

}
#endif
