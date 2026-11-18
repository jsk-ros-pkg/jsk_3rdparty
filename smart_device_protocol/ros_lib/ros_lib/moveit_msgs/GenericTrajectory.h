#ifndef _ROS_moveit_msgs_GenericTrajectory_h
#define _ROS_moveit_msgs_GenericTrajectory_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "trajectory_msgs/JointTrajectory.h"
#include "moveit_msgs/CartesianTrajectory.h"

namespace moveit_msgs
{

  class GenericTrajectory : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      uint32_t joint_trajectory_length;
      typedef trajectory_msgs::JointTrajectory _joint_trajectory_type;
      _joint_trajectory_type st_joint_trajectory;
      _joint_trajectory_type * joint_trajectory;
      uint32_t cartesian_trajectory_length;
      typedef moveit_msgs::CartesianTrajectory _cartesian_trajectory_type;
      _cartesian_trajectory_type st_cartesian_trajectory;
      _cartesian_trajectory_type * cartesian_trajectory;

    GenericTrajectory():
      header(),
      joint_trajectory_length(0), st_joint_trajectory(), joint_trajectory(nullptr),
      cartesian_trajectory_length(0), st_cartesian_trajectory(), cartesian_trajectory(nullptr)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      *(outbuffer + offset + 0) = (this->joint_trajectory_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->joint_trajectory_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->joint_trajectory_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->joint_trajectory_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->joint_trajectory_length);
      for( uint32_t i = 0; i < joint_trajectory_length; i++){
      offset += this->joint_trajectory[i].serialize(outbuffer + offset);
      }
      *(outbuffer + offset + 0) = (this->cartesian_trajectory_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->cartesian_trajectory_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->cartesian_trajectory_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->cartesian_trajectory_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->cartesian_trajectory_length);
      for( uint32_t i = 0; i < cartesian_trajectory_length; i++){
      offset += this->cartesian_trajectory[i].serialize(outbuffer + offset);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      uint32_t joint_trajectory_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      joint_trajectory_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      joint_trajectory_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      joint_trajectory_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->joint_trajectory_length);
      if(joint_trajectory_lengthT > joint_trajectory_length)
        this->joint_trajectory = (trajectory_msgs::JointTrajectory*)realloc(this->joint_trajectory, joint_trajectory_lengthT * sizeof(trajectory_msgs::JointTrajectory));
      joint_trajectory_length = joint_trajectory_lengthT;
      for( uint32_t i = 0; i < joint_trajectory_length; i++){
      offset += this->st_joint_trajectory.deserialize(inbuffer + offset);
        memcpy( &(this->joint_trajectory[i]), &(this->st_joint_trajectory), sizeof(trajectory_msgs::JointTrajectory));
      }
      uint32_t cartesian_trajectory_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      cartesian_trajectory_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      cartesian_trajectory_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      cartesian_trajectory_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->cartesian_trajectory_length);
      if(cartesian_trajectory_lengthT > cartesian_trajectory_length)
        this->cartesian_trajectory = (moveit_msgs::CartesianTrajectory*)realloc(this->cartesian_trajectory, cartesian_trajectory_lengthT * sizeof(moveit_msgs::CartesianTrajectory));
      cartesian_trajectory_length = cartesian_trajectory_lengthT;
      for( uint32_t i = 0; i < cartesian_trajectory_length; i++){
      offset += this->st_cartesian_trajectory.deserialize(inbuffer + offset);
        memcpy( &(this->cartesian_trajectory[i]), &(this->st_cartesian_trajectory), sizeof(moveit_msgs::CartesianTrajectory));
      }
     return offset;
    }

    virtual const char * getType() override { return "moveit_msgs/GenericTrajectory"; };
    virtual const char * getMD5() override { return "d68b5c73072efa2012238a77e49c2c58"; };

  };

}
#endif
