#ifndef _ROS_SERVICE_ProjectFootstep_h
#define _ROS_SERVICE_ProjectFootstep_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "jsk_footstep_msgs/FootstepArray.h"

namespace jsk_footstep_planner
{

static const char PROJECTFOOTSTEP[] = "jsk_footstep_planner/ProjectFootstep";

  class ProjectFootstepRequest : public ros::Msg
  {
    public:
      typedef uint8_t _project_type_type;
      _project_type_type project_type;
      typedef jsk_footstep_msgs::FootstepArray _input_type;
      _input_type input;

    ProjectFootstepRequest():
      project_type(0),
      input()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->project_type >> (8 * 0)) & 0xFF;
      offset += sizeof(this->project_type);
      offset += this->input.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      this->project_type =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->project_type);
      offset += this->input.deserialize(inbuffer + offset);
     return offset;
    }

    virtual const char * getType() override { return PROJECTFOOTSTEP; };
    virtual const char * getMD5() override { return "aa8e4e9b4dd5cb69940dd879b6fecb96"; };

  };

  class ProjectFootstepResponse : public ros::Msg
  {
    public:
      uint32_t success_length;
      typedef bool _success_type;
      _success_type st_success;
      _success_type * success;
      typedef jsk_footstep_msgs::FootstepArray _result_type;
      _result_type result;

    ProjectFootstepResponse():
      success_length(0), st_success(), success(nullptr),
      result()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->success_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->success_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->success_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->success_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->success_length);
      for( uint32_t i = 0; i < success_length; i++){
      union {
        bool real;
        uint8_t base;
      } u_successi;
      u_successi.real = this->success[i];
      *(outbuffer + offset + 0) = (u_successi.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->success[i]);
      }
      offset += this->result.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      uint32_t success_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      success_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      success_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      success_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->success_length);
      if(success_lengthT > success_length)
        this->success = (bool*)realloc(this->success, success_lengthT * sizeof(bool));
      success_length = success_lengthT;
      for( uint32_t i = 0; i < success_length; i++){
      union {
        bool real;
        uint8_t base;
      } u_st_success;
      u_st_success.base = 0;
      u_st_success.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->st_success = u_st_success.real;
      offset += sizeof(this->st_success);
        memcpy( &(this->success[i]), &(this->st_success), sizeof(bool));
      }
      offset += this->result.deserialize(inbuffer + offset);
     return offset;
    }

    virtual const char * getType() override { return PROJECTFOOTSTEP; };
    virtual const char * getMD5() override { return "0e35d9b6cc53f98167d0eeaf015660dc"; };

  };

  class ProjectFootstep {
    public:
    typedef ProjectFootstepRequest Request;
    typedef ProjectFootstepResponse Response;
  };

}
#endif
