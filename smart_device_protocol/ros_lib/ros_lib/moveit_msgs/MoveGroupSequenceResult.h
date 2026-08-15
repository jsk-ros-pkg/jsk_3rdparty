#ifndef _ROS_moveit_msgs_MoveGroupSequenceResult_h
#define _ROS_moveit_msgs_MoveGroupSequenceResult_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "moveit_msgs/MotionSequenceResponse.h"

namespace moveit_msgs
{

  class MoveGroupSequenceResult : public ros::Msg
  {
    public:
      typedef moveit_msgs::MotionSequenceResponse _response_type;
      _response_type response;

    MoveGroupSequenceResult():
      response()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->response.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += this->response.deserialize(inbuffer + offset);
     return offset;
    }

    virtual const char * getType() override { return "moveit_msgs/MoveGroupSequenceResult"; };
    virtual const char * getMD5() override { return "3e3d83067566e443fa885c9428941f17"; };

  };

}
#endif
