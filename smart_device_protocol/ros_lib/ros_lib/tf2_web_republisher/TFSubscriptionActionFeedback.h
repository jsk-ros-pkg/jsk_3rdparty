#ifndef _ROS_tf2_web_republisher_TFSubscriptionActionFeedback_h
#define _ROS_tf2_web_republisher_TFSubscriptionActionFeedback_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "actionlib_msgs/GoalStatus.h"
#include "tf2_web_republisher/TFSubscriptionFeedback.h"

namespace tf2_web_republisher
{

  class TFSubscriptionActionFeedback : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef actionlib_msgs::GoalStatus _status_type;
      _status_type status;
      typedef tf2_web_republisher::TFSubscriptionFeedback _feedback_type;
      _feedback_type feedback;

    TFSubscriptionActionFeedback():
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

    virtual const char * getType() override { return "tf2_web_republisher/TFSubscriptionActionFeedback"; };
    virtual const char * getMD5() override { return "de686654be3ef0f8970616dd702bb360"; };

  };

}
#endif
