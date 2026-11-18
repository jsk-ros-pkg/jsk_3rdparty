#ifndef _ROS_roseus_smach_Sub5ActionFeedback_h
#define _ROS_roseus_smach_Sub5ActionFeedback_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "actionlib_msgs/GoalStatus.h"
#include "roseus_smach/Sub5Feedback.h"

namespace roseus_smach
{

  class Sub5ActionFeedback : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef actionlib_msgs::GoalStatus _status_type;
      _status_type status;
      typedef roseus_smach::Sub5Feedback _feedback_type;
      _feedback_type feedback;

    Sub5ActionFeedback():
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

    virtual const char * getType() override { return "roseus_smach/Sub5ActionFeedback"; };
    virtual const char * getMD5() override { return "ce7bcc860ef310bc2edbb3a3a2388663"; };

  };

}
#endif
