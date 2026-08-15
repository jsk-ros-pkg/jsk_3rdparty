#ifndef _ROS_moveit_msgs_RobotTrajectory_h
#define _ROS_moveit_msgs_RobotTrajectory_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "trajectory_msgs/JointTrajectory.h"
#include "trajectory_msgs/MultiDOFJointTrajectory.h"

namespace moveit_msgs
{

  class RobotTrajectory : public ros::Msg
  {
    public:
      typedef trajectory_msgs::JointTrajectory _joint_trajectory_type;
      _joint_trajectory_type joint_trajectory;
      typedef trajectory_msgs::MultiDOFJointTrajectory _multi_dof_joint_trajectory_type;
      _multi_dof_joint_trajectory_type multi_dof_joint_trajectory;

    RobotTrajectory():
      joint_trajectory(),
      multi_dof_joint_trajectory()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->joint_trajectory.serialize(outbuffer + offset);
      offset += this->multi_dof_joint_trajectory.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += this->joint_trajectory.deserialize(inbuffer + offset);
      offset += this->multi_dof_joint_trajectory.deserialize(inbuffer + offset);
     return offset;
    }

    virtual const char * getType() override { return "moveit_msgs/RobotTrajectory"; };
    virtual const char * getMD5() override { return "dfa9556423d709a3729bcef664bddf67"; };

  };

}
#endif
