#ifndef _ROS_grasping_msgs_FindGraspableObjectsActionResult_h
#define _ROS_grasping_msgs_FindGraspableObjectsActionResult_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "actionlib_msgs/GoalStatus.h"
#include "grasping_msgs/FindGraspableObjectsResult.h"

namespace grasping_msgs
{

  class FindGraspableObjectsActionResult : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef actionlib_msgs::GoalStatus _status_type;
      _status_type status;
      typedef grasping_msgs::FindGraspableObjectsResult _result_type;
      _result_type result;

    FindGraspableObjectsActionResult():
      header(),
      status(),
      result()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      offset += this->status.serialize(outbuffer + offset);
      offset += this->result.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      offset += this->status.deserialize(inbuffer + offset);
      offset += this->result.deserialize(inbuffer + offset);
     return offset;
    }

    virtual const char * getType() override { return "grasping_msgs/FindGraspableObjectsActionResult"; };
    virtual const char * getMD5() override { return "815c3872a35e011e303fdc6f38a6dc62"; };

  };

}
#endif
