#ifndef _ROS_SERVICE_SetModelsJointsStates_h
#define _ROS_SERVICE_SetModelsJointsStates_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "pr2_gazebo_plugins/ModelJointsState.h"

namespace pr2_gazebo_plugins
{

static const char SETMODELSJOINTSSTATES[] = "pr2_gazebo_plugins/SetModelsJointsStates";

  class SetModelsJointsStatesRequest : public ros::Msg
  {
    public:
      uint32_t model_names_length;
      typedef char* _model_names_type;
      _model_names_type st_model_names;
      _model_names_type * model_names;
      uint32_t model_joints_states_length;
      typedef pr2_gazebo_plugins::ModelJointsState _model_joints_states_type;
      _model_joints_states_type st_model_joints_states;
      _model_joints_states_type * model_joints_states;

    SetModelsJointsStatesRequest():
      model_names_length(0), st_model_names(), model_names(nullptr),
      model_joints_states_length(0), st_model_joints_states(), model_joints_states(nullptr)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->model_names_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->model_names_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->model_names_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->model_names_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->model_names_length);
      for( uint32_t i = 0; i < model_names_length; i++){
      uint32_t length_model_namesi = strlen(this->model_names[i]);
      varToArr(outbuffer + offset, length_model_namesi);
      offset += 4;
      memcpy(outbuffer + offset, this->model_names[i], length_model_namesi);
      offset += length_model_namesi;
      }
      *(outbuffer + offset + 0) = (this->model_joints_states_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->model_joints_states_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->model_joints_states_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->model_joints_states_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->model_joints_states_length);
      for( uint32_t i = 0; i < model_joints_states_length; i++){
      offset += this->model_joints_states[i].serialize(outbuffer + offset);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      uint32_t model_names_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      model_names_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      model_names_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      model_names_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->model_names_length);
      if(model_names_lengthT > model_names_length)
        this->model_names = (char**)realloc(this->model_names, model_names_lengthT * sizeof(char*));
      model_names_length = model_names_lengthT;
      for( uint32_t i = 0; i < model_names_length; i++){
      uint32_t length_st_model_names;
      arrToVar(length_st_model_names, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_st_model_names; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_st_model_names-1]=0;
      this->st_model_names = (char *)(inbuffer + offset-1);
      offset += length_st_model_names;
        memcpy( &(this->model_names[i]), &(this->st_model_names), sizeof(char*));
      }
      uint32_t model_joints_states_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      model_joints_states_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      model_joints_states_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      model_joints_states_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->model_joints_states_length);
      if(model_joints_states_lengthT > model_joints_states_length)
        this->model_joints_states = (pr2_gazebo_plugins::ModelJointsState*)realloc(this->model_joints_states, model_joints_states_lengthT * sizeof(pr2_gazebo_plugins::ModelJointsState));
      model_joints_states_length = model_joints_states_lengthT;
      for( uint32_t i = 0; i < model_joints_states_length; i++){
      offset += this->st_model_joints_states.deserialize(inbuffer + offset);
        memcpy( &(this->model_joints_states[i]), &(this->st_model_joints_states), sizeof(pr2_gazebo_plugins::ModelJointsState));
      }
     return offset;
    }

    virtual const char * getType() override { return SETMODELSJOINTSSTATES; };
    virtual const char * getMD5() override { return "ecf71b483df7b70447575a8231727200"; };

  };

  class SetModelsJointsStatesResponse : public ros::Msg
  {
    public:
      typedef bool _success_type;
      _success_type success;
      typedef const char* _status_message_type;
      _status_message_type status_message;

    SetModelsJointsStatesResponse():
      success(0),
      status_message("")
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_success;
      u_success.real = this->success;
      *(outbuffer + offset + 0) = (u_success.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->success);
      uint32_t length_status_message = strlen(this->status_message);
      varToArr(outbuffer + offset, length_status_message);
      offset += 4;
      memcpy(outbuffer + offset, this->status_message, length_status_message);
      offset += length_status_message;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_success;
      u_success.base = 0;
      u_success.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->success = u_success.real;
      offset += sizeof(this->success);
      uint32_t length_status_message;
      arrToVar(length_status_message, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_status_message; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_status_message-1]=0;
      this->status_message = (char *)(inbuffer + offset-1);
      offset += length_status_message;
     return offset;
    }

    virtual const char * getType() override { return SETMODELSJOINTSSTATES; };
    virtual const char * getMD5() override { return "2ec6f3eff0161f4257b808b12bc830c2"; };

  };

  class SetModelsJointsStates {
    public:
    typedef SetModelsJointsStatesRequest Request;
    typedef SetModelsJointsStatesResponse Response;
  };

}
#endif
