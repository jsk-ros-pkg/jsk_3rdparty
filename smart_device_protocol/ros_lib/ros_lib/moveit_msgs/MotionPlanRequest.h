#ifndef _ROS_moveit_msgs_MotionPlanRequest_h
#define _ROS_moveit_msgs_MotionPlanRequest_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "moveit_msgs/WorkspaceParameters.h"
#include "moveit_msgs/RobotState.h"
#include "moveit_msgs/Constraints.h"
#include "moveit_msgs/TrajectoryConstraints.h"
#include "moveit_msgs/GenericTrajectory.h"

namespace moveit_msgs
{

  class MotionPlanRequest : public ros::Msg
  {
    public:
      typedef moveit_msgs::WorkspaceParameters _workspace_parameters_type;
      _workspace_parameters_type workspace_parameters;
      typedef moveit_msgs::RobotState _start_state_type;
      _start_state_type start_state;
      uint32_t goal_constraints_length;
      typedef moveit_msgs::Constraints _goal_constraints_type;
      _goal_constraints_type st_goal_constraints;
      _goal_constraints_type * goal_constraints;
      typedef moveit_msgs::Constraints _path_constraints_type;
      _path_constraints_type path_constraints;
      typedef moveit_msgs::TrajectoryConstraints _trajectory_constraints_type;
      _trajectory_constraints_type trajectory_constraints;
      uint32_t reference_trajectories_length;
      typedef moveit_msgs::GenericTrajectory _reference_trajectories_type;
      _reference_trajectories_type st_reference_trajectories;
      _reference_trajectories_type * reference_trajectories;
      typedef const char* _pipeline_id_type;
      _pipeline_id_type pipeline_id;
      typedef const char* _planner_id_type;
      _planner_id_type planner_id;
      typedef const char* _group_name_type;
      _group_name_type group_name;
      typedef int32_t _num_planning_attempts_type;
      _num_planning_attempts_type num_planning_attempts;
      typedef float _allowed_planning_time_type;
      _allowed_planning_time_type allowed_planning_time;
      typedef float _max_velocity_scaling_factor_type;
      _max_velocity_scaling_factor_type max_velocity_scaling_factor;
      typedef float _max_acceleration_scaling_factor_type;
      _max_acceleration_scaling_factor_type max_acceleration_scaling_factor;
      typedef const char* _cartesian_speed_limited_link_type;
      _cartesian_speed_limited_link_type cartesian_speed_limited_link;
      typedef float _max_cartesian_speed_type;
      _max_cartesian_speed_type max_cartesian_speed;

    MotionPlanRequest():
      workspace_parameters(),
      start_state(),
      goal_constraints_length(0), st_goal_constraints(), goal_constraints(nullptr),
      path_constraints(),
      trajectory_constraints(),
      reference_trajectories_length(0), st_reference_trajectories(), reference_trajectories(nullptr),
      pipeline_id(""),
      planner_id(""),
      group_name(""),
      num_planning_attempts(0),
      allowed_planning_time(0),
      max_velocity_scaling_factor(0),
      max_acceleration_scaling_factor(0),
      cartesian_speed_limited_link(""),
      max_cartesian_speed(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->workspace_parameters.serialize(outbuffer + offset);
      offset += this->start_state.serialize(outbuffer + offset);
      *(outbuffer + offset + 0) = (this->goal_constraints_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->goal_constraints_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->goal_constraints_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->goal_constraints_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->goal_constraints_length);
      for( uint32_t i = 0; i < goal_constraints_length; i++){
      offset += this->goal_constraints[i].serialize(outbuffer + offset);
      }
      offset += this->path_constraints.serialize(outbuffer + offset);
      offset += this->trajectory_constraints.serialize(outbuffer + offset);
      *(outbuffer + offset + 0) = (this->reference_trajectories_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->reference_trajectories_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->reference_trajectories_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->reference_trajectories_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->reference_trajectories_length);
      for( uint32_t i = 0; i < reference_trajectories_length; i++){
      offset += this->reference_trajectories[i].serialize(outbuffer + offset);
      }
      uint32_t length_pipeline_id = strlen(this->pipeline_id);
      varToArr(outbuffer + offset, length_pipeline_id);
      offset += 4;
      memcpy(outbuffer + offset, this->pipeline_id, length_pipeline_id);
      offset += length_pipeline_id;
      uint32_t length_planner_id = strlen(this->planner_id);
      varToArr(outbuffer + offset, length_planner_id);
      offset += 4;
      memcpy(outbuffer + offset, this->planner_id, length_planner_id);
      offset += length_planner_id;
      uint32_t length_group_name = strlen(this->group_name);
      varToArr(outbuffer + offset, length_group_name);
      offset += 4;
      memcpy(outbuffer + offset, this->group_name, length_group_name);
      offset += length_group_name;
      union {
        int32_t real;
        uint32_t base;
      } u_num_planning_attempts;
      u_num_planning_attempts.real = this->num_planning_attempts;
      *(outbuffer + offset + 0) = (u_num_planning_attempts.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_num_planning_attempts.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_num_planning_attempts.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_num_planning_attempts.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->num_planning_attempts);
      offset += serializeAvrFloat64(outbuffer + offset, this->allowed_planning_time);
      offset += serializeAvrFloat64(outbuffer + offset, this->max_velocity_scaling_factor);
      offset += serializeAvrFloat64(outbuffer + offset, this->max_acceleration_scaling_factor);
      uint32_t length_cartesian_speed_limited_link = strlen(this->cartesian_speed_limited_link);
      varToArr(outbuffer + offset, length_cartesian_speed_limited_link);
      offset += 4;
      memcpy(outbuffer + offset, this->cartesian_speed_limited_link, length_cartesian_speed_limited_link);
      offset += length_cartesian_speed_limited_link;
      offset += serializeAvrFloat64(outbuffer + offset, this->max_cartesian_speed);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += this->workspace_parameters.deserialize(inbuffer + offset);
      offset += this->start_state.deserialize(inbuffer + offset);
      uint32_t goal_constraints_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      goal_constraints_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      goal_constraints_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      goal_constraints_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->goal_constraints_length);
      if(goal_constraints_lengthT > goal_constraints_length)
        this->goal_constraints = (moveit_msgs::Constraints*)realloc(this->goal_constraints, goal_constraints_lengthT * sizeof(moveit_msgs::Constraints));
      goal_constraints_length = goal_constraints_lengthT;
      for( uint32_t i = 0; i < goal_constraints_length; i++){
      offset += this->st_goal_constraints.deserialize(inbuffer + offset);
        memcpy( &(this->goal_constraints[i]), &(this->st_goal_constraints), sizeof(moveit_msgs::Constraints));
      }
      offset += this->path_constraints.deserialize(inbuffer + offset);
      offset += this->trajectory_constraints.deserialize(inbuffer + offset);
      uint32_t reference_trajectories_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      reference_trajectories_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      reference_trajectories_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      reference_trajectories_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->reference_trajectories_length);
      if(reference_trajectories_lengthT > reference_trajectories_length)
        this->reference_trajectories = (moveit_msgs::GenericTrajectory*)realloc(this->reference_trajectories, reference_trajectories_lengthT * sizeof(moveit_msgs::GenericTrajectory));
      reference_trajectories_length = reference_trajectories_lengthT;
      for( uint32_t i = 0; i < reference_trajectories_length; i++){
      offset += this->st_reference_trajectories.deserialize(inbuffer + offset);
        memcpy( &(this->reference_trajectories[i]), &(this->st_reference_trajectories), sizeof(moveit_msgs::GenericTrajectory));
      }
      uint32_t length_pipeline_id;
      arrToVar(length_pipeline_id, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_pipeline_id; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_pipeline_id-1]=0;
      this->pipeline_id = (char *)(inbuffer + offset-1);
      offset += length_pipeline_id;
      uint32_t length_planner_id;
      arrToVar(length_planner_id, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_planner_id; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_planner_id-1]=0;
      this->planner_id = (char *)(inbuffer + offset-1);
      offset += length_planner_id;
      uint32_t length_group_name;
      arrToVar(length_group_name, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_group_name; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_group_name-1]=0;
      this->group_name = (char *)(inbuffer + offset-1);
      offset += length_group_name;
      union {
        int32_t real;
        uint32_t base;
      } u_num_planning_attempts;
      u_num_planning_attempts.base = 0;
      u_num_planning_attempts.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_num_planning_attempts.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_num_planning_attempts.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_num_planning_attempts.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->num_planning_attempts = u_num_planning_attempts.real;
      offset += sizeof(this->num_planning_attempts);
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->allowed_planning_time));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->max_velocity_scaling_factor));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->max_acceleration_scaling_factor));
      uint32_t length_cartesian_speed_limited_link;
      arrToVar(length_cartesian_speed_limited_link, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_cartesian_speed_limited_link; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_cartesian_speed_limited_link-1]=0;
      this->cartesian_speed_limited_link = (char *)(inbuffer + offset-1);
      offset += length_cartesian_speed_limited_link;
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->max_cartesian_speed));
     return offset;
    }

    virtual const char * getType() override { return "moveit_msgs/MotionPlanRequest"; };
    virtual const char * getMD5() override { return "83f3d7b1d47a538f7659e0de9dae7980"; };

  };

}
#endif
