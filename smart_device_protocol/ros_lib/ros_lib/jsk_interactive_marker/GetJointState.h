#ifndef _ROS_SERVICE_GetJointState_h
#define _ROS_SERVICE_GetJointState_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "sensor_msgs/JointState.h"

namespace jsk_interactive_marker
{

static const char GETJOINTSTATE[] = "jsk_interactive_marker/GetJointState";

  class GetJointStateRequest : public ros::Msg
  {
    public:

    GetJointStateRequest()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
     return offset;
    }

    virtual const char * getType() override { return GETJOINTSTATE; };
    virtual const char * getMD5() override { return "d41d8cd98f00b204e9800998ecf8427e"; };

  };

  class GetJointStateResponse : public ros::Msg
  {
    public:
      typedef sensor_msgs::JointState _joint_state_type;
      _joint_state_type joint_state;

    GetJointStateResponse():
      joint_state()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->joint_state.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += this->joint_state.deserialize(inbuffer + offset);
     return offset;
    }

    virtual const char * getType() override { return GETJOINTSTATE; };
    virtual const char * getMD5() override { return "9ca061465ef0ed08771ed240c43789f5"; };

  };

  class GetJointState {
    public:
    typedef GetJointStateRequest Request;
    typedef GetJointStateResponse Response;
  };

}
#endif
