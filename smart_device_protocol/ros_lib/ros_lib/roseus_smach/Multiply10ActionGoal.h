#ifndef _ROS_roseus_smach_Multiply10ActionGoal_h
#define _ROS_roseus_smach_Multiply10ActionGoal_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "actionlib_msgs/GoalID.h"
#include "roseus_smach/Multiply10Goal.h"

namespace roseus_smach
{

  class Multiply10ActionGoal : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef actionlib_msgs::GoalID _goal_id_type;
      _goal_id_type goal_id;
      typedef roseus_smach::Multiply10Goal _goal_type;
      _goal_type goal;

    Multiply10ActionGoal():
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

    virtual const char * getType() override { return "roseus_smach/Multiply10ActionGoal"; };
    virtual const char * getMD5() override { return "dc1a763911ac8fed593b103b6d986502"; };

  };

}
#endif
