#ifndef _ROS_jsk_interactive_marker_MoveModel_h
#define _ROS_jsk_interactive_marker_MoveModel_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "sensor_msgs/JointState.h"
#include "geometry_msgs/PoseStamped.h"

namespace jsk_interactive_marker
{

  class MoveModel : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef const char* _name_type;
      _name_type name;
      typedef const char* _description_type;
      _description_type description;
      typedef sensor_msgs::JointState _joint_state_origin_type;
      _joint_state_origin_type joint_state_origin;
      typedef sensor_msgs::JointState _joint_state_goal_type;
      _joint_state_goal_type joint_state_goal;
      typedef geometry_msgs::PoseStamped _pose_origin_type;
      _pose_origin_type pose_origin;
      typedef geometry_msgs::PoseStamped _pose_goal_type;
      _pose_goal_type pose_goal;

    MoveModel():
      header(),
      name(""),
      description(""),
      joint_state_origin(),
      joint_state_goal(),
      pose_origin(),
      pose_goal()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      uint32_t length_name = strlen(this->name);
      varToArr(outbuffer + offset, length_name);
      offset += 4;
      memcpy(outbuffer + offset, this->name, length_name);
      offset += length_name;
      uint32_t length_description = strlen(this->description);
      varToArr(outbuffer + offset, length_description);
      offset += 4;
      memcpy(outbuffer + offset, this->description, length_description);
      offset += length_description;
      offset += this->joint_state_origin.serialize(outbuffer + offset);
      offset += this->joint_state_goal.serialize(outbuffer + offset);
      offset += this->pose_origin.serialize(outbuffer + offset);
      offset += this->pose_goal.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      uint32_t length_name;
      arrToVar(length_name, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_name; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_name-1]=0;
      this->name = (char *)(inbuffer + offset-1);
      offset += length_name;
      uint32_t length_description;
      arrToVar(length_description, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_description; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_description-1]=0;
      this->description = (char *)(inbuffer + offset-1);
      offset += length_description;
      offset += this->joint_state_origin.deserialize(inbuffer + offset);
      offset += this->joint_state_goal.deserialize(inbuffer + offset);
      offset += this->pose_origin.deserialize(inbuffer + offset);
      offset += this->pose_goal.deserialize(inbuffer + offset);
     return offset;
    }

    virtual const char * getType() override { return "jsk_interactive_marker/MoveModel"; };
    virtual const char * getMD5() override { return "61edcddd6ca50e876e68d4fcf06c90f6"; };

  };

}
#endif
