#ifndef _ROS_robot_controllers_msgs_QueryControllerStatesActionFeedback_h
#define _ROS_robot_controllers_msgs_QueryControllerStatesActionFeedback_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "actionlib_msgs/GoalStatus.h"
#include "robot_controllers_msgs/QueryControllerStatesFeedback.h"

namespace robot_controllers_msgs
{

  class QueryControllerStatesActionFeedback : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef actionlib_msgs::GoalStatus _status_type;
      _status_type status;
      typedef robot_controllers_msgs::QueryControllerStatesFeedback _feedback_type;
      _feedback_type feedback;

    QueryControllerStatesActionFeedback():
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

    virtual const char * getType() override { return "robot_controllers_msgs/QueryControllerStatesActionFeedback"; };
    virtual const char * getMD5() override { return "27d9ffea72fe024e2f600b8921a3f631"; };

  };

}
#endif
