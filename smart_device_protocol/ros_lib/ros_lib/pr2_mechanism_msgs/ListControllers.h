#ifndef _ROS_SERVICE_ListControllers_h
#define _ROS_SERVICE_ListControllers_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace pr2_mechanism_msgs
{

static const char LISTCONTROLLERS[] = "pr2_mechanism_msgs/ListControllers";

  class ListControllersRequest : public ros::Msg
  {
    public:

    ListControllersRequest()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
     return offset;
    }

    virtual const char * getType() override { return LISTCONTROLLERS; };
    virtual const char * getMD5() override { return "d41d8cd98f00b204e9800998ecf8427e"; };

  };

  class ListControllersResponse : public ros::Msg
  {
    public:
      uint32_t controllers_length;
      typedef char* _controllers_type;
      _controllers_type st_controllers;
      _controllers_type * controllers;
      uint32_t state_length;
      typedef char* _state_type;
      _state_type st_state;
      _state_type * state;

    ListControllersResponse():
      controllers_length(0), st_controllers(), controllers(nullptr),
      state_length(0), st_state(), state(nullptr)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->controllers_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->controllers_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->controllers_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->controllers_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->controllers_length);
      for( uint32_t i = 0; i < controllers_length; i++){
      uint32_t length_controllersi = strlen(this->controllers[i]);
      varToArr(outbuffer + offset, length_controllersi);
      offset += 4;
      memcpy(outbuffer + offset, this->controllers[i], length_controllersi);
      offset += length_controllersi;
      }
      *(outbuffer + offset + 0) = (this->state_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->state_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->state_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->state_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->state_length);
      for( uint32_t i = 0; i < state_length; i++){
      uint32_t length_statei = strlen(this->state[i]);
      varToArr(outbuffer + offset, length_statei);
      offset += 4;
      memcpy(outbuffer + offset, this->state[i], length_statei);
      offset += length_statei;
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      uint32_t controllers_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      controllers_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      controllers_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      controllers_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->controllers_length);
      if(controllers_lengthT > controllers_length)
        this->controllers = (char**)realloc(this->controllers, controllers_lengthT * sizeof(char*));
      controllers_length = controllers_lengthT;
      for( uint32_t i = 0; i < controllers_length; i++){
      uint32_t length_st_controllers;
      arrToVar(length_st_controllers, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_st_controllers; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_st_controllers-1]=0;
      this->st_controllers = (char *)(inbuffer + offset-1);
      offset += length_st_controllers;
        memcpy( &(this->controllers[i]), &(this->st_controllers), sizeof(char*));
      }
      uint32_t state_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      state_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      state_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      state_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->state_length);
      if(state_lengthT > state_length)
        this->state = (char**)realloc(this->state, state_lengthT * sizeof(char*));
      state_length = state_lengthT;
      for( uint32_t i = 0; i < state_length; i++){
      uint32_t length_st_state;
      arrToVar(length_st_state, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_st_state; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_st_state-1]=0;
      this->st_state = (char *)(inbuffer + offset-1);
      offset += length_st_state;
        memcpy( &(this->state[i]), &(this->st_state), sizeof(char*));
      }
     return offset;
    }

    virtual const char * getType() override { return LISTCONTROLLERS; };
    virtual const char * getMD5() override { return "39c8d39516aed5c7d76284ac06c220e5"; };

  };

  class ListControllers {
    public:
    typedef ListControllersRequest Request;
    typedef ListControllersResponse Response;
  };

}
#endif
