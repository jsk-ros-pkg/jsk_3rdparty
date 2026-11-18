#ifndef _ROS_naoqi_bridge_msgs_SpeechWithFeedbackActionGoal_h
#define _ROS_naoqi_bridge_msgs_SpeechWithFeedbackActionGoal_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "actionlib_msgs/GoalID.h"
#include "naoqi_bridge_msgs/SpeechWithFeedbackGoal.h"

namespace naoqi_bridge_msgs
{

  class SpeechWithFeedbackActionGoal : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef actionlib_msgs::GoalID _goal_id_type;
      _goal_id_type goal_id;
      typedef naoqi_bridge_msgs::SpeechWithFeedbackGoal _goal_type;
      _goal_type goal;

    SpeechWithFeedbackActionGoal():
      header(),
      goal_id(),
      goal()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      offset += this->goal_id.serialize(outbuffer + offset);
      offset += this->goal.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      offset += this->goal_id.deserialize(inbuffer + offset);
      offset += this->goal.deserialize(inbuffer + offset);
     return offset;
    }

    virtual const char * getType() override { return "naoqi_bridge_msgs/SpeechWithFeedbackActionGoal"; };
    virtual const char * getMD5() override { return "5d34431dc14f96985d41d1835bc895fe"; };

  };

}
#endif
