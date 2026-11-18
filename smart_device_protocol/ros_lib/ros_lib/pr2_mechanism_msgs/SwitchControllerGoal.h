#ifndef _ROS_pr2_mechanism_msgs_SwitchControllerGoal_h
#define _ROS_pr2_mechanism_msgs_SwitchControllerGoal_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace pr2_mechanism_msgs
{

  class SwitchControllerGoal : public ros::Msg
  {
    public:
      uint32_t start_controllers_length;
      typedef char* _start_controllers_type;
      _start_controllers_type st_start_controllers;
      _start_controllers_type * start_controllers;
      uint32_t stop_controllers_length;
      typedef char* _stop_controllers_type;
      _stop_controllers_type st_stop_controllers;
      _stop_controllers_type * stop_controllers;

    SwitchControllerGoal():
      start_controllers_length(0), st_start_controllers(), start_controllers(nullptr),
      stop_controllers_length(0), st_stop_controllers(), stop_controllers(nullptr)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->start_controllers_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->start_controllers_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->start_controllers_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->start_controllers_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->start_controllers_length);
      for( uint32_t i = 0; i < start_controllers_length; i++){
      uint32_t length_start_controllersi = strlen(this->start_controllers[i]);
      varToArr(outbuffer + offset, length_start_controllersi);
      offset += 4;
      memcpy(outbuffer + offset, this->start_controllers[i], length_start_controllersi);
      offset += length_start_controllersi;
      }
      *(outbuffer + offset + 0) = (this->stop_controllers_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->stop_controllers_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->stop_controllers_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->stop_controllers_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->stop_controllers_length);
      for( uint32_t i = 0; i < stop_controllers_length; i++){
      uint32_t length_stop_controllersi = strlen(this->stop_controllers[i]);
      varToArr(outbuffer + offset, length_stop_controllersi);
      offset += 4;
      memcpy(outbuffer + offset, this->stop_controllers[i], length_stop_controllersi);
      offset += length_stop_controllersi;
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      uint32_t start_controllers_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      start_controllers_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      start_controllers_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      start_controllers_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->start_controllers_length);
      if(start_controllers_lengthT > start_controllers_length)
        this->start_controllers = (char**)realloc(this->start_controllers, start_controllers_lengthT * sizeof(char*));
      start_controllers_length = start_controllers_lengthT;
      for( uint32_t i = 0; i < start_controllers_length; i++){
      uint32_t length_st_start_controllers;
      arrToVar(length_st_start_controllers, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_st_start_controllers; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_st_start_controllers-1]=0;
      this->st_start_controllers = (char *)(inbuffer + offset-1);
      offset += length_st_start_controllers;
        memcpy( &(this->start_controllers[i]), &(this->st_start_controllers), sizeof(char*));
      }
      uint32_t stop_controllers_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      stop_controllers_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      stop_controllers_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      stop_controllers_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->stop_controllers_length);
      if(stop_controllers_lengthT > stop_controllers_length)
        this->stop_controllers = (char**)realloc(this->stop_controllers, stop_controllers_lengthT * sizeof(char*));
      stop_controllers_length = stop_controllers_lengthT;
      for( uint32_t i = 0; i < stop_controllers_length; i++){
      uint32_t length_st_stop_controllers;
      arrToVar(length_st_stop_controllers, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_st_stop_controllers; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_st_stop_controllers-1]=0;
      this->st_stop_controllers = (char *)(inbuffer + offset-1);
      offset += length_st_stop_controllers;
        memcpy( &(this->stop_controllers[i]), &(this->st_stop_controllers), sizeof(char*));
      }
     return offset;
    }

    virtual const char * getType() override { return "pr2_mechanism_msgs/SwitchControllerGoal"; };
    virtual const char * getMD5() override { return "c3f1879cbb2d6cd8732c0e031574dde5"; };

  };

}
#endif
