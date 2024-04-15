#ifndef _ROS_pr2_gripper_sensor_msgs_PR2GripperReleaseCommand_h
#define _ROS_pr2_gripper_sensor_msgs_PR2GripperReleaseCommand_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "pr2_gripper_sensor_msgs/PR2GripperEventDetectorCommand.h"

namespace pr2_gripper_sensor_msgs
{

  class PR2GripperReleaseCommand : public ros::Msg
  {
    public:
      typedef pr2_gripper_sensor_msgs::PR2GripperEventDetectorCommand _event_type;
      _event_type event;

    PR2GripperReleaseCommand():
      event()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->event.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += this->event.deserialize(inbuffer + offset);
     return offset;
    }

    virtual const char * getType() override { return "pr2_gripper_sensor_msgs/PR2GripperReleaseCommand"; };
    virtual const char * getMD5() override { return "e62b08129864bf301ed0a1335e6158dc"; };

  };

}
#endif
