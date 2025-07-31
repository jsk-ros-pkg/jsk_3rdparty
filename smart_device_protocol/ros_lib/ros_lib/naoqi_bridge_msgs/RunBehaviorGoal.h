#ifndef _ROS_naoqi_bridge_msgs_RunBehaviorGoal_h
#define _ROS_naoqi_bridge_msgs_RunBehaviorGoal_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace naoqi_bridge_msgs
{

  class RunBehaviorGoal : public ros::Msg
  {
    public:
      typedef const char* _behavior_type;
      _behavior_type behavior;

    RunBehaviorGoal():
      behavior("")
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      uint32_t length_behavior = strlen(this->behavior);
      varToArr(outbuffer + offset, length_behavior);
      offset += 4;
      memcpy(outbuffer + offset, this->behavior, length_behavior);
      offset += length_behavior;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      uint32_t length_behavior;
      arrToVar(length_behavior, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_behavior; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_behavior-1]=0;
      this->behavior = (char *)(inbuffer + offset-1);
      offset += length_behavior;
     return offset;
    }

    virtual const char * getType() override { return "naoqi_bridge_msgs/RunBehaviorGoal"; };
    virtual const char * getMD5() override { return "03729983c4b9be7a4f2b56846a7ccbdc"; };

  };

}
#endif
