#ifndef _ROS_jsk_footstep_msgs_PlanFootstepsResult_h
#define _ROS_jsk_footstep_msgs_PlanFootstepsResult_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "jsk_footstep_msgs/FootstepArray.h"

namespace jsk_footstep_msgs
{

  class PlanFootstepsResult : public ros::Msg
  {
    public:
      typedef jsk_footstep_msgs::FootstepArray _result_type;
      _result_type result;

    PlanFootstepsResult():
      result()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->result.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += this->result.deserialize(inbuffer + offset);
     return offset;
    }

    virtual const char * getType() override { return "jsk_footstep_msgs/PlanFootstepsResult"; };
    virtual const char * getMD5() override { return "951c674749b5b8b08d5eba2e3e62aedd"; };

  };

}
#endif
