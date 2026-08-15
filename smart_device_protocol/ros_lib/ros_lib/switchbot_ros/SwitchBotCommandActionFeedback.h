#ifndef _ROS_switchbot_ros_SwitchBotCommandActionFeedback_h
#define _ROS_switchbot_ros_SwitchBotCommandActionFeedback_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "actionlib_msgs/GoalStatus.h"
#include "switchbot_ros/SwitchBotCommandFeedback.h"

namespace switchbot_ros
{

  class SwitchBotCommandActionFeedback : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef actionlib_msgs::GoalStatus _status_type;
      _status_type status;
      typedef switchbot_ros::SwitchBotCommandFeedback _feedback_type;
      _feedback_type feedback;

    SwitchBotCommandActionFeedback():
      header(),
      status(),
      feedback()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      offset += this->status.serialize(outbuffer + offset);
      offset += this->feedback.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      offset += this->status.deserialize(inbuffer + offset);
      offset += this->feedback.deserialize(inbuffer + offset);
     return offset;
    }

    virtual const char * getType() override { return "switchbot_ros/SwitchBotCommandActionFeedback"; };
    virtual const char * getMD5() override { return "82e97615735594b25e2cee3da22d3eb7"; };

  };

}
#endif
