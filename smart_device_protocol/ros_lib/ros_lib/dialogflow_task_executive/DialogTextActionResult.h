#ifndef _ROS_dialogflow_task_executive_DialogTextActionResult_h
#define _ROS_dialogflow_task_executive_DialogTextActionResult_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "actionlib_msgs/GoalStatus.h"
#include "dialogflow_task_executive/DialogTextResult.h"

namespace dialogflow_task_executive
{

  class DialogTextActionResult : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef actionlib_msgs::GoalStatus _status_type;
      _status_type status;
      typedef dialogflow_task_executive::DialogTextResult _result_type;
      _result_type result;

    DialogTextActionResult():
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

    virtual const char * getType() override { return "dialogflow_task_executive/DialogTextActionResult"; };
    virtual const char * getMD5() override { return "064bee682b09e4fb44617e4f8739bcf5"; };

  };

}
#endif
