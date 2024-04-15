#ifndef _ROS_fetch_auto_dock_msgs_DockActionGoal_h
#define _ROS_fetch_auto_dock_msgs_DockActionGoal_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "actionlib_msgs/GoalID.h"
#include "fetch_auto_dock_msgs/DockGoal.h"

namespace fetch_auto_dock_msgs
{

  class DockActionGoal : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef actionlib_msgs::GoalID _goal_id_type;
      _goal_id_type goal_id;
      typedef fetch_auto_dock_msgs::DockGoal _goal_type;
      _goal_type goal;

    DockActionGoal():
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

    virtual const char * getType() override { return "fetch_auto_dock_msgs/DockActionGoal"; };
    virtual const char * getMD5() override { return "69333baaf59d995d4a69e40ea04b8312"; };

  };

}
#endif
