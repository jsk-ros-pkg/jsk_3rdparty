#ifndef _ROS_pr2_common_action_msgs_TuckArmsActionResult_h
#define _ROS_pr2_common_action_msgs_TuckArmsActionResult_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "actionlib_msgs/GoalStatus.h"
#include "pr2_common_action_msgs/TuckArmsResult.h"

namespace pr2_common_action_msgs
{

  class TuckArmsActionResult : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef actionlib_msgs::GoalStatus _status_type;
      _status_type status;
      typedef pr2_common_action_msgs::TuckArmsResult _result_type;
      _result_type result;

    TuckArmsActionResult():
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

    virtual const char * getType() override { return "pr2_common_action_msgs/TuckArmsActionResult"; };
    virtual const char * getMD5() override { return "a151ea69df95c9525872b19d347d7f8e"; };

  };

}
#endif
