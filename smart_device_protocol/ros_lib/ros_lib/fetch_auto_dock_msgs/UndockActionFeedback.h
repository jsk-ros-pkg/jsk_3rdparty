#ifndef _ROS_fetch_auto_dock_msgs_UndockActionFeedback_h
#define _ROS_fetch_auto_dock_msgs_UndockActionFeedback_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "actionlib_msgs/GoalStatus.h"
#include "fetch_auto_dock_msgs/UndockFeedback.h"

namespace fetch_auto_dock_msgs
{

  class UndockActionFeedback : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef actionlib_msgs::GoalStatus _status_type;
      _status_type status;
      typedef fetch_auto_dock_msgs::UndockFeedback _feedback_type;
      _feedback_type feedback;

    UndockActionFeedback():
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

    virtual const char * getType() override { return "fetch_auto_dock_msgs/UndockActionFeedback"; };
    virtual const char * getMD5() override { return "6a07b7150fca355bea027d8c95ed9e67"; };

  };

}
#endif
