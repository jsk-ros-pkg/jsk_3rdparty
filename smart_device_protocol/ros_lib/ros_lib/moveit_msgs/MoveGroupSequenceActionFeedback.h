#ifndef _ROS_moveit_msgs_MoveGroupSequenceActionFeedback_h
#define _ROS_moveit_msgs_MoveGroupSequenceActionFeedback_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "actionlib_msgs/GoalStatus.h"
#include "moveit_msgs/MoveGroupSequenceFeedback.h"

namespace moveit_msgs
{

  class MoveGroupSequenceActionFeedback : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef actionlib_msgs::GoalStatus _status_type;
      _status_type status;
      typedef moveit_msgs::MoveGroupSequenceFeedback _feedback_type;
      _feedback_type feedback;

    MoveGroupSequenceActionFeedback():
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

    virtual const char * getType() override { return "moveit_msgs/MoveGroupSequenceActionFeedback"; };
    virtual const char * getMD5() override { return "12232ef97486c7962f264c105aae2958"; };

  };

}
#endif
