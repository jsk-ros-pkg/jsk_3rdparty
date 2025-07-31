#ifndef _ROS_moveit_msgs_MoveGroupGoal_h
#define _ROS_moveit_msgs_MoveGroupGoal_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "moveit_msgs/MotionPlanRequest.h"
#include "moveit_msgs/PlanningOptions.h"

namespace moveit_msgs
{

  class MoveGroupGoal : public ros::Msg
  {
    public:
      typedef moveit_msgs::MotionPlanRequest _request_type;
      _request_type request;
      typedef moveit_msgs::PlanningOptions _planning_options_type;
      _planning_options_type planning_options;

    MoveGroupGoal():
      request(),
      planning_options()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->request.serialize(outbuffer + offset);
      offset += this->planning_options.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += this->request.deserialize(inbuffer + offset);
      offset += this->planning_options.deserialize(inbuffer + offset);
     return offset;
    }

    virtual const char * getType() override { return "moveit_msgs/MoveGroupGoal"; };
    virtual const char * getMD5() override { return "ce7f9820670af166b2faa14c475891b0"; };

  };

}
#endif
