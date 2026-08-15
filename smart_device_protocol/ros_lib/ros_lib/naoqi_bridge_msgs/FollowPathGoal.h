#ifndef _ROS_naoqi_bridge_msgs_FollowPathGoal_h
#define _ROS_naoqi_bridge_msgs_FollowPathGoal_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "nav_msgs/Path.h"

namespace naoqi_bridge_msgs
{

  class FollowPathGoal : public ros::Msg
  {
    public:
      typedef nav_msgs::Path _path_type;
      _path_type path;

    FollowPathGoal():
      path()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->path.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += this->path.deserialize(inbuffer + offset);
     return offset;
    }

    virtual const char * getType() override { return "naoqi_bridge_msgs/FollowPathGoal"; };
    virtual const char * getMD5() override { return "58d6f138c7de7ef47c75d4b7e5df5472"; };

  };

}
#endif
