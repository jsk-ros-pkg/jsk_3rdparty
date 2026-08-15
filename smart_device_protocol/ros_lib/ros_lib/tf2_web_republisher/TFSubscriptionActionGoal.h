#ifndef _ROS_tf2_web_republisher_TFSubscriptionActionGoal_h
#define _ROS_tf2_web_republisher_TFSubscriptionActionGoal_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "actionlib_msgs/GoalID.h"
#include "tf2_web_republisher/TFSubscriptionGoal.h"

namespace tf2_web_republisher
{

  class TFSubscriptionActionGoal : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef actionlib_msgs::GoalID _goal_id_type;
      _goal_id_type goal_id;
      typedef tf2_web_republisher::TFSubscriptionGoal _goal_type;
      _goal_type goal;

    TFSubscriptionActionGoal():
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

    virtual const char * getType() override { return "tf2_web_republisher/TFSubscriptionActionGoal"; };
    virtual const char * getMD5() override { return "ef8da891ba3ba9b13d97bca8154eaeb5"; };

  };

}
#endif
