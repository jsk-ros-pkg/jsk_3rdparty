#ifndef _ROS_SERVICE_SetPlannerParams_h
#define _ROS_SERVICE_SetPlannerParams_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "moveit_msgs/PlannerParams.h"

namespace moveit_msgs
{

static const char SETPLANNERPARAMS[] = "moveit_msgs/SetPlannerParams";

  class SetPlannerParamsRequest : public ros::Msg
  {
    public:
      typedef const char* _pipeline_id_type;
      _pipeline_id_type pipeline_id;
      typedef const char* _planner_config_type;
      _planner_config_type planner_config;
      typedef const char* _group_type;
      _group_type group;
      typedef moveit_msgs::PlannerParams _params_type;
      _params_type params;
      typedef bool _replace_type;
      _replace_type replace;

    SetPlannerParamsRequest():
      pipeline_id(""),
      planner_config(""),
      group(""),
      params(),
      replace(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      uint32_t length_pipeline_id = strlen(this->pipeline_id);
      varToArr(outbuffer + offset, length_pipeline_id);
      offset += 4;
      memcpy(outbuffer + offset, this->pipeline_id, length_pipeline_id);
      offset += length_pipeline_id;
      uint32_t length_planner_config = strlen(this->planner_config);
      varToArr(outbuffer + offset, length_planner_config);
      offset += 4;
      memcpy(outbuffer + offset, this->planner_config, length_planner_config);
      offset += length_planner_config;
      uint32_t length_group = strlen(this->group);
      varToArr(outbuffer + offset, length_group);
      offset += 4;
      memcpy(outbuffer + offset, this->group, length_group);
      offset += length_group;
      offset += this->params.serialize(outbuffer + offset);
      union {
        bool real;
        uint8_t base;
      } u_replace;
      u_replace.real = this->replace;
      *(outbuffer + offset + 0) = (u_replace.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->replace);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      uint32_t length_pipeline_id;
      arrToVar(length_pipeline_id, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_pipeline_id; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_pipeline_id-1]=0;
      this->pipeline_id = (char *)(inbuffer + offset-1);
      offset += length_pipeline_id;
      uint32_t length_planner_config;
      arrToVar(length_planner_config, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_planner_config; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_planner_config-1]=0;
      this->planner_config = (char *)(inbuffer + offset-1);
      offset += length_planner_config;
      uint32_t length_group;
      arrToVar(length_group, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_group; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_group-1]=0;
      this->group = (char *)(inbuffer + offset-1);
      offset += length_group;
      offset += this->params.deserialize(inbuffer + offset);
      union {
        bool real;
        uint8_t base;
      } u_replace;
      u_replace.base = 0;
      u_replace.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->replace = u_replace.real;
      offset += sizeof(this->replace);
     return offset;
    }

    virtual const char * getType() override { return SETPLANNERPARAMS; };
    virtual const char * getMD5() override { return "14bebee5d4d53a2df94b7f146d3eb2ff"; };

  };

  class SetPlannerParamsResponse : public ros::Msg
  {
    public:

    SetPlannerParamsResponse()
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

    virtual const char * getType() override { return SETPLANNERPARAMS; };
    virtual const char * getMD5() override { return "d41d8cd98f00b204e9800998ecf8427e"; };

  };

  class SetPlannerParams {
    public:
    typedef SetPlannerParamsRequest Request;
    typedef SetPlannerParamsResponse Response;
  };

}
#endif
