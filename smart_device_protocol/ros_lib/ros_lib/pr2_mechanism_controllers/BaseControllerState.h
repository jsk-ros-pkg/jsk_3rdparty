#ifndef _ROS_pr2_mechanism_controllers_BaseControllerState_h
#define _ROS_pr2_mechanism_controllers_BaseControllerState_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "geometry_msgs/Twist.h"

namespace pr2_mechanism_controllers
{

  class BaseControllerState : public ros::Msg
  {
    public:
      typedef geometry_msgs::Twist _command_type;
      _command_type command;
      uint32_t joint_velocity_measured_length;
      typedef float _joint_velocity_measured_type;
      _joint_velocity_measured_type st_joint_velocity_measured;
      _joint_velocity_measured_type * joint_velocity_measured;
      uint32_t joint_velocity_commanded_length;
      typedef float _joint_velocity_commanded_type;
      _joint_velocity_commanded_type st_joint_velocity_commanded;
      _joint_velocity_commanded_type * joint_velocity_commanded;
      uint32_t joint_velocity_error_length;
      typedef float _joint_velocity_error_type;
      _joint_velocity_error_type st_joint_velocity_error;
      _joint_velocity_error_type * joint_velocity_error;
      uint32_t joint_effort_measured_length;
      typedef float _joint_effort_measured_type;
      _joint_effort_measured_type st_joint_effort_measured;
      _joint_effort_measured_type * joint_effort_measured;
      uint32_t joint_effort_commanded_length;
      typedef float _joint_effort_commanded_type;
      _joint_effort_commanded_type st_joint_effort_commanded;
      _joint_effort_commanded_type * joint_effort_commanded;
      uint32_t joint_effort_error_length;
      typedef float _joint_effort_error_type;
      _joint_effort_error_type st_joint_effort_error;
      _joint_effort_error_type * joint_effort_error;
      uint32_t joint_names_length;
      typedef char* _joint_names_type;
      _joint_names_type st_joint_names;
      _joint_names_type * joint_names;

    BaseControllerState():
      command(),
      joint_velocity_measured_length(0), st_joint_velocity_measured(), joint_velocity_measured(nullptr),
      joint_velocity_commanded_length(0), st_joint_velocity_commanded(), joint_velocity_commanded(nullptr),
      joint_velocity_error_length(0), st_joint_velocity_error(), joint_velocity_error(nullptr),
      joint_effort_measured_length(0), st_joint_effort_measured(), joint_effort_measured(nullptr),
      joint_effort_commanded_length(0), st_joint_effort_commanded(), joint_effort_commanded(nullptr),
      joint_effort_error_length(0), st_joint_effort_error(), joint_effort_error(nullptr),
      joint_names_length(0), st_joint_names(), joint_names(nullptr)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->command.serialize(outbuffer + offset);
      *(outbuffer + offset + 0) = (this->joint_velocity_measured_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->joint_velocity_measured_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->joint_velocity_measured_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->joint_velocity_measured_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->joint_velocity_measured_length);
      for( uint32_t i = 0; i < joint_velocity_measured_length; i++){
      offset += serializeAvrFloat64(outbuffer + offset, this->joint_velocity_measured[i]);
      }
      *(outbuffer + offset + 0) = (this->joint_velocity_commanded_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->joint_velocity_commanded_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->joint_velocity_commanded_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->joint_velocity_commanded_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->joint_velocity_commanded_length);
      for( uint32_t i = 0; i < joint_velocity_commanded_length; i++){
      offset += serializeAvrFloat64(outbuffer + offset, this->joint_velocity_commanded[i]);
      }
      *(outbuffer + offset + 0) = (this->joint_velocity_error_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->joint_velocity_error_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->joint_velocity_error_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->joint_velocity_error_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->joint_velocity_error_length);
      for( uint32_t i = 0; i < joint_velocity_error_length; i++){
      offset += serializeAvrFloat64(outbuffer + offset, this->joint_velocity_error[i]);
      }
      *(outbuffer + offset + 0) = (this->joint_effort_measured_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->joint_effort_measured_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->joint_effort_measured_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->joint_effort_measured_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->joint_effort_measured_length);
      for( uint32_t i = 0; i < joint_effort_measured_length; i++){
      offset += serializeAvrFloat64(outbuffer + offset, this->joint_effort_measured[i]);
      }
      *(outbuffer + offset + 0) = (this->joint_effort_commanded_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->joint_effort_commanded_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->joint_effort_commanded_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->joint_effort_commanded_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->joint_effort_commanded_length);
      for( uint32_t i = 0; i < joint_effort_commanded_length; i++){
      offset += serializeAvrFloat64(outbuffer + offset, this->joint_effort_commanded[i]);
      }
      *(outbuffer + offset + 0) = (this->joint_effort_error_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->joint_effort_error_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->joint_effort_error_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->joint_effort_error_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->joint_effort_error_length);
      for( uint32_t i = 0; i < joint_effort_error_length; i++){
      offset += serializeAvrFloat64(outbuffer + offset, this->joint_effort_error[i]);
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
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += this->command.deserialize(inbuffer + offset);
      uint32_t joint_velocity_measured_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      joint_velocity_measured_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      joint_velocity_measured_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      joint_velocity_measured_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->joint_velocity_measured_length);
      if(joint_velocity_measured_lengthT > joint_velocity_measured_length)
        this->joint_velocity_measured = (float*)realloc(this->joint_velocity_measured, joint_velocity_measured_lengthT * sizeof(float));
      joint_velocity_measured_length = joint_velocity_measured_lengthT;
      for( uint32_t i = 0; i < joint_velocity_measured_length; i++){
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->st_joint_velocity_measured));
        memcpy( &(this->joint_velocity_measured[i]), &(this->st_joint_velocity_measured), sizeof(float));
      }
      uint32_t joint_velocity_commanded_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      joint_velocity_commanded_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      joint_velocity_commanded_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      joint_velocity_commanded_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->joint_velocity_commanded_length);
      if(joint_velocity_commanded_lengthT > joint_velocity_commanded_length)
        this->joint_velocity_commanded = (float*)realloc(this->joint_velocity_commanded, joint_velocity_commanded_lengthT * sizeof(float));
      joint_velocity_commanded_length = joint_velocity_commanded_lengthT;
      for( uint32_t i = 0; i < joint_velocity_commanded_length; i++){
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->st_joint_velocity_commanded));
        memcpy( &(this->joint_velocity_commanded[i]), &(this->st_joint_velocity_commanded), sizeof(float));
      }
      uint32_t joint_velocity_error_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      joint_velocity_error_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      joint_velocity_error_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      joint_velocity_error_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->joint_velocity_error_length);
      if(joint_velocity_error_lengthT > joint_velocity_error_length)
        this->joint_velocity_error = (float*)realloc(this->joint_velocity_error, joint_velocity_error_lengthT * sizeof(float));
      joint_velocity_error_length = joint_velocity_error_lengthT;
      for( uint32_t i = 0; i < joint_velocity_error_length; i++){
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->st_joint_velocity_error));
        memcpy( &(this->joint_velocity_error[i]), &(this->st_joint_velocity_error), sizeof(float));
      }
      uint32_t joint_effort_measured_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      joint_effort_measured_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      joint_effort_measured_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      joint_effort_measured_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->joint_effort_measured_length);
      if(joint_effort_measured_lengthT > joint_effort_measured_length)
        this->joint_effort_measured = (float*)realloc(this->joint_effort_measured, joint_effort_measured_lengthT * sizeof(float));
      joint_effort_measured_length = joint_effort_measured_lengthT;
      for( uint32_t i = 0; i < joint_effort_measured_length; i++){
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->st_joint_effort_measured));
        memcpy( &(this->joint_effort_measured[i]), &(this->st_joint_effort_measured), sizeof(float));
      }
      uint32_t joint_effort_commanded_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      joint_effort_commanded_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      joint_effort_commanded_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      joint_effort_commanded_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->joint_effort_commanded_length);
      if(joint_effort_commanded_lengthT > joint_effort_commanded_length)
        this->joint_effort_commanded = (float*)realloc(this->joint_effort_commanded, joint_effort_commanded_lengthT * sizeof(float));
      joint_effort_commanded_length = joint_effort_commanded_lengthT;
      for( uint32_t i = 0; i < joint_effort_commanded_length; i++){
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->st_joint_effort_commanded));
        memcpy( &(this->joint_effort_commanded[i]), &(this->st_joint_effort_commanded), sizeof(float));
      }
      uint32_t joint_effort_error_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      joint_effort_error_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      joint_effort_error_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      joint_effort_error_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->joint_effort_error_length);
      if(joint_effort_error_lengthT > joint_effort_error_length)
        this->joint_effort_error = (float*)realloc(this->joint_effort_error, joint_effort_error_lengthT * sizeof(float));
      joint_effort_error_length = joint_effort_error_lengthT;
      for( uint32_t i = 0; i < joint_effort_error_length; i++){
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->st_joint_effort_error));
        memcpy( &(this->joint_effort_error[i]), &(this->st_joint_effort_error), sizeof(float));
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
     return offset;
    }

    virtual const char * getType() override { return "pr2_mechanism_controllers/BaseControllerState"; };
    virtual const char * getMD5() override { return "7a488aa492f9175d5fa35e22e56c4b28"; };

  };

}
#endif
