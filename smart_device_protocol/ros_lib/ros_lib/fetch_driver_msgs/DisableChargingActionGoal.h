#ifndef _ROS_fetch_driver_msgs_DisableChargingActionGoal_h
#define _ROS_fetch_driver_msgs_DisableChargingActionGoal_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "actionlib_msgs/GoalID.h"
#include "fetch_driver_msgs/DisableChargingGoal.h"

namespace fetch_driver_msgs
{

  class DisableChargingActionGoal : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef actionlib_msgs::GoalID _goal_id_type;
      _goal_id_type goal_id;
      typedef fetch_driver_msgs::DisableChargingGoal _goal_type;
      _goal_type goal;

    DisableChargingActionGoal():
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

    virtual const char * getType() override { return "fetch_driver_msgs/DisableChargingActionGoal"; };
    virtual const char * getMD5() override { return "2cdef171810817cf95b4cd420433b17f"; };

  };

}
#endif
