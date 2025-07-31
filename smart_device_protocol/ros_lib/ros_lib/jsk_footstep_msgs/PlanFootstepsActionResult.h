#ifndef _ROS_jsk_footstep_msgs_PlanFootstepsActionResult_h
#define _ROS_jsk_footstep_msgs_PlanFootstepsActionResult_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "actionlib_msgs/GoalStatus.h"
#include "jsk_footstep_msgs/PlanFootstepsResult.h"

namespace jsk_footstep_msgs
{

  class PlanFootstepsActionResult : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef actionlib_msgs::GoalStatus _status_type;
      _status_type status;
      typedef jsk_footstep_msgs::PlanFootstepsResult _result_type;
      _result_type result;

    PlanFootstepsActionResult():
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

    virtual const char * getType() override { return "jsk_footstep_msgs/PlanFootstepsActionResult"; };
    virtual const char * getMD5() override { return "12a9334ae88f7e7881a5ffdea1dd3de1"; };

  };

}
#endif
