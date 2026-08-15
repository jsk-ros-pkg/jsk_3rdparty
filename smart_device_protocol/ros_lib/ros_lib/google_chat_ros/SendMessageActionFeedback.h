#ifndef _ROS_google_chat_ros_SendMessageActionFeedback_h
#define _ROS_google_chat_ros_SendMessageActionFeedback_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "actionlib_msgs/GoalStatus.h"
#include "google_chat_ros/SendMessageFeedback.h"

namespace google_chat_ros
{

  class SendMessageActionFeedback : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef actionlib_msgs::GoalStatus _status_type;
      _status_type status;
      typedef google_chat_ros::SendMessageFeedback _feedback_type;
      _feedback_type feedback;

    SendMessageActionFeedback():
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

    virtual const char * getType() override { return "google_chat_ros/SendMessageActionFeedback"; };
    virtual const char * getMD5() override { return "82e97615735594b25e2cee3da22d3eb7"; };

  };

}
#endif
