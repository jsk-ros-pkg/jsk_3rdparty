#ifndef _ROS_grasping_msgs_FindGraspableObjectsFeedback_h
#define _ROS_grasping_msgs_FindGraspableObjectsFeedback_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "grasping_msgs/GraspableObject.h"

namespace grasping_msgs
{

  class FindGraspableObjectsFeedback : public ros::Msg
  {
    public:
      typedef grasping_msgs::GraspableObject _object_type;
      _object_type object;

    FindGraspableObjectsFeedback():
      object()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->object.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += this->object.deserialize(inbuffer + offset);
     return offset;
    }

    virtual const char * getType() override { return "grasping_msgs/FindGraspableObjectsFeedback"; };
    virtual const char * getMD5() override { return "64c6bfc02f7e1c6e2d2473d1c1329ec7"; };

  };

}
#endif
