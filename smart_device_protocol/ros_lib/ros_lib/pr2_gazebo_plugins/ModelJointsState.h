#ifndef _ROS_pr2_gazebo_plugins_ModelJointsState_h
#define _ROS_pr2_gazebo_plugins_ModelJointsState_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "geometry_msgs/Pose.h"

namespace pr2_gazebo_plugins
{

  class ModelJointsState : public ros::Msg
  {
    public:
      uint32_t model_pose_length;
      typedef geometry_msgs::Pose _model_pose_type;
      _model_pose_type st_model_pose;
      _model_pose_type * model_pose;
      uint32_t joint_names_length;
      typedef char* _joint_names_type;
      _joint_names_type st_joint_names;
      _joint_names_type * joint_names;
      uint32_t joint_positions_length;
      typedef float _joint_positions_type;
      _joint_positions_type st_joint_positions;
      _joint_positions_type * joint_positions;

    ModelJointsState():
      model_pose_length(0), st_model_pose(), model_pose(nullptr),
      joint_names_length(0), st_joint_names(), joint_names(nullptr),
      joint_positions_length(0), st_joint_positions(), joint_positions(nullptr)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->model_pose_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->model_pose_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->model_pose_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->model_pose_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->model_pose_length);
      for( uint32_t i = 0; i < model_pose_length; i++){
      offset += this->model_pose[i].serialize(outbuffer + offset);
      }
      *(outbuffer + offset + 0) = (this->joint_names_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->joint_names_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->joint_names_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->joint_names_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->joint_names_length);
      for( uint32_t i = 0; i < joint_names_length; i++){
      uint32_t length_joint_namesi = strlen(this->joint_names[i]);
      varToArr(outbuffer + offset, length_joint_namesi);
      offset += 4;
      memcpy(outbuffer + offset, this->joint_names[i], length_joint_namesi);
      offset += length_joint_namesi;
      }
      *(outbuffer + offset + 0) = (this->joint_positions_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->joint_positions_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->joint_positions_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->joint_positions_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->joint_positions_length);
      for( uint32_t i = 0; i < joint_positions_length; i++){
      offset += serializeAvrFloat64(outbuffer + offset, this->joint_positions[i]);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      uint32_t model_pose_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      model_pose_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      model_pose_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      model_pose_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->model_pose_length);
      if(model_pose_lengthT > model_pose_length)
        this->model_pose = (geometry_msgs::Pose*)realloc(this->model_pose, model_pose_lengthT * sizeof(geometry_msgs::Pose));
      model_pose_length = model_pose_lengthT;
      for( uint32_t i = 0; i < model_pose_length; i++){
      offset += this->st_model_pose.deserialize(inbuffer + offset);
        memcpy( &(this->model_pose[i]), &(this->st_model_pose), sizeof(geometry_msgs::Pose));
      }
      uint32_t joint_names_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      joint_names_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      joint_names_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      joint_names_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->joint_names_length);
      if(joint_names_lengthT > joint_names_length)
        this->joint_names = (char**)realloc(this->joint_names, joint_names_lengthT * sizeof(char*));
      joint_names_length = joint_names_lengthT;
      for( uint32_t i = 0; i < joint_names_length; i++){
      uint32_t length_st_joint_names;
      arrToVar(length_st_joint_names, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_st_joint_names; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_st_joint_names-1]=0;
      this->st_joint_names = (char *)(inbuffer + offset-1);
      offset += length_st_joint_names;
        memcpy( &(this->joint_names[i]), &(this->st_joint_names), sizeof(char*));
      }
      uint32_t joint_positions_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      joint_positions_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      joint_positions_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      joint_positions_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->joint_positions_length);
      if(joint_positions_lengthT > joint_positions_length)
        this->joint_positions = (float*)realloc(this->joint_positions, joint_positions_lengthT * sizeof(float));
      joint_positions_length = joint_positions_lengthT;
      for( uint32_t i = 0; i < joint_positions_length; i++){
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->st_joint_positions));
        memcpy( &(this->joint_positions[i]), &(this->st_joint_positions), sizeof(float));
      }
     return offset;
    }

    virtual const char * getType() override { return "pr2_gazebo_plugins/ModelJointsState"; };
    virtual const char * getMD5() override { return "f700a74958b6566fae4cd77fbb80ffd4"; };

  };

}
#endif
