#ifndef _ROS_SERVICE_GetCartesianPath_h
#define _ROS_SERVICE_GetCartesianPath_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "moveit_msgs/Constraints.h"
#include "std_msgs/Header.h"
#include "moveit_msgs/RobotTrajectory.h"
#include "moveit_msgs/RobotState.h"
#include "geometry_msgs/Pose.h"
#include "moveit_msgs/MoveItErrorCodes.h"

namespace moveit_msgs
{

static const char GETCARTESIANPATH[] = "moveit_msgs/GetCartesianPath";

  class GetCartesianPathRequest : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef moveit_msgs::RobotState _start_state_type;
      _start_state_type start_state;
      typedef const char* _group_name_type;
      _group_name_type group_name;
      typedef const char* _link_name_type;
      _link_name_type link_name;
      uint32_t waypoints_length;
      typedef geometry_msgs::Pose _waypoints_type;
      _waypoints_type st_waypoints;
      _waypoints_type * waypoints;
      typedef float _max_step_type;
      _max_step_type max_step;
      typedef float _jump_threshold_type;
      _jump_threshold_type jump_threshold;
      typedef float _prismatic_jump_threshold_type;
      _prismatic_jump_threshold_type prismatic_jump_threshold;
      typedef float _revolute_jump_threshold_type;
      _revolute_jump_threshold_type revolute_jump_threshold;
      typedef bool _avoid_collisions_type;
      _avoid_collisions_type avoid_collisions;
      typedef moveit_msgs::Constraints _path_constraints_type;
      _path_constraints_type path_constraints;
      typedef const char* _cartesian_speed_limited_link_type;
      _cartesian_speed_limited_link_type cartesian_speed_limited_link;
      typedef float _max_cartesian_speed_type;
      _max_cartesian_speed_type max_cartesian_speed;

    GetCartesianPathRequest():
      header(),
      start_state(),
      group_name(""),
      link_name(""),
      waypoints_length(0), st_waypoints(), waypoints(nullptr),
      max_step(0),
      jump_threshold(0),
      prismatic_jump_threshold(0),
      revolute_jump_threshold(0),
      avoid_collisions(0),
      path_constraints(),
      cartesian_speed_limited_link(""),
      max_cartesian_speed(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      offset += this->start_state.serialize(outbuffer + offset);
      uint32_t length_group_name = strlen(this->group_name);
      varToArr(outbuffer + offset, length_group_name);
      offset += 4;
      memcpy(outbuffer + offset, this->group_name, length_group_name);
      offset += length_group_name;
      uint32_t length_link_name = strlen(this->link_name);
      varToArr(outbuffer + offset, length_link_name);
      offset += 4;
      memcpy(outbuffer + offset, this->link_name, length_link_name);
      offset += length_link_name;
      *(outbuffer + offset + 0) = (this->waypoints_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->waypoints_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->waypoints_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->waypoints_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->waypoints_length);
      for( uint32_t i = 0; i < waypoints_length; i++){
      offset += this->waypoints[i].serialize(outbuffer + offset);
      }
      offset += serializeAvrFloat64(outbuffer + offset, this->max_step);
      offset += serializeAvrFloat64(outbuffer + offset, this->jump_threshold);
      offset += serializeAvrFloat64(outbuffer + offset, this->prismatic_jump_threshold);
      offset += serializeAvrFloat64(outbuffer + offset, this->revolute_jump_threshold);
      union {
        bool real;
        uint8_t base;
      } u_avoid_collisions;
      u_avoid_collisions.real = this->avoid_collisions;
      *(outbuffer + offset + 0) = (u_avoid_collisions.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->avoid_collisions);
      offset += this->path_constraints.serialize(outbuffer + offset);
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
      offset += this->header.deserialize(inbuffer + offset);
      offset += this->start_state.deserialize(inbuffer + offset);
      uint32_t length_group_name;
      arrToVar(length_group_name, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_group_name; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_group_name-1]=0;
      this->group_name = (char *)(inbuffer + offset-1);
      offset += length_group_name;
      uint32_t length_link_name;
      arrToVar(length_link_name, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_link_name; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_link_name-1]=0;
      this->link_name = (char *)(inbuffer + offset-1);
      offset += length_link_name;
      uint32_t waypoints_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      waypoints_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      waypoints_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      waypoints_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->waypoints_length);
      if(waypoints_lengthT > waypoints_length)
        this->waypoints = (geometry_msgs::Pose*)realloc(this->waypoints, waypoints_lengthT * sizeof(geometry_msgs::Pose));
      waypoints_length = waypoints_lengthT;
      for( uint32_t i = 0; i < waypoints_length; i++){
      offset += this->st_waypoints.deserialize(inbuffer + offset);
        memcpy( &(this->waypoints[i]), &(this->st_waypoints), sizeof(geometry_msgs::Pose));
      }
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->max_step));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->jump_threshold));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->prismatic_jump_threshold));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->revolute_jump_threshold));
      union {
        bool real;
        uint8_t base;
      } u_avoid_collisions;
      u_avoid_collisions.base = 0;
      u_avoid_collisions.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->avoid_collisions = u_avoid_collisions.real;
      offset += sizeof(this->avoid_collisions);
      offset += this->path_constraints.deserialize(inbuffer + offset);
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

    virtual const char * getType() override { return GETCARTESIANPATH; };
    virtual const char * getMD5() override { return "97b21c683d2b97f701099048039a05af"; };

  };

  class GetCartesianPathResponse : public ros::Msg
  {
    public:
      typedef moveit_msgs::RobotState _start_state_type;
      _start_state_type start_state;
      typedef moveit_msgs::RobotTrajectory _solution_type;
      _solution_type solution;
      typedef float _fraction_type;
      _fraction_type fraction;
      typedef moveit_msgs::MoveItErrorCodes _error_code_type;
      _error_code_type error_code;

    GetCartesianPathResponse():
      start_state(),
      solution(),
      fraction(0),
      error_code()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->start_state.serialize(outbuffer + offset);
      offset += this->solution.serialize(outbuffer + offset);
      offset += serializeAvrFloat64(outbuffer + offset, this->fraction);
      offset += this->error_code.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += this->start_state.deserialize(inbuffer + offset);
      offset += this->solution.deserialize(inbuffer + offset);
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->fraction));
      offset += this->error_code.deserialize(inbuffer + offset);
     return offset;
    }

    virtual const char * getType() override { return GETCARTESIANPATH; };
    virtual const char * getMD5() override { return "13da68a622269d4097c7ad732fb4a27f"; };

  };

  class GetCartesianPath {
    public:
    typedef GetCartesianPathRequest Request;
    typedef GetCartesianPathResponse Response;
  };

}
#endif
