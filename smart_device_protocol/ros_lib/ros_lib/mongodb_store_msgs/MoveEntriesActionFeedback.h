#ifndef _ROS_mongodb_store_msgs_MoveEntriesActionFeedback_h
#define _ROS_mongodb_store_msgs_MoveEntriesActionFeedback_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "actionlib_msgs/GoalStatus.h"
#include "mongodb_store_msgs/MoveEntriesFeedback.h"

namespace mongodb_store_msgs
{

  class MoveEntriesActionFeedback : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef actionlib_msgs::GoalStatus _status_type;
      _status_type status;
      typedef mongodb_store_msgs::MoveEntriesFeedback _feedback_type;
      _feedback_type feedback;

    MoveEntriesActionFeedback():
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

    virtual const char * getType() override { return "mongodb_store_msgs/MoveEntriesActionFeedback"; };
    virtual const char * getMD5() override { return "6dafd6ecae79e05a072ce163776da775"; };

  };

}
#endif
