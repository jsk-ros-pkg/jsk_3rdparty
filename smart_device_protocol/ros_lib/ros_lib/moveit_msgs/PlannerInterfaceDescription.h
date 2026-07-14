#ifndef _ROS_moveit_msgs_PlannerInterfaceDescription_h
#define _ROS_moveit_msgs_PlannerInterfaceDescription_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace moveit_msgs
{

  class PlannerInterfaceDescription : public ros::Msg
  {
    public:
      typedef const char* _name_type;
      _name_type name;
      typedef const char* _pipeline_id_type;
      _pipeline_id_type pipeline_id;
      uint32_t planner_ids_length;
      typedef char* _planner_ids_type;
      _planner_ids_type st_planner_ids;
      _planner_ids_type * planner_ids;

    PlannerInterfaceDescription():
      name(""),
      pipeline_id(""),
      planner_ids_length(0), st_planner_ids(), planner_ids(nullptr)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      uint32_t length_name = strlen(this->name);
      varToArr(outbuffer + offset, length_name);
      offset += 4;
      memcpy(outbuffer + offset, this->name, length_name);
      offset += length_name;
      uint32_t length_pipeline_id = strlen(this->pipeline_id);
      varToArr(outbuffer + offset, length_pipeline_id);
      offset += 4;
      memcpy(outbuffer + offset, this->pipeline_id, length_pipeline_id);
      offset += length_pipeline_id;
      *(outbuffer + offset + 0) = (this->planner_ids_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->planner_ids_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->planner_ids_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->planner_ids_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->planner_ids_length);
      for( uint32_t i = 0; i < planner_ids_length; i++){
      uint32_t length_planner_idsi = strlen(this->planner_ids[i]);
      varToArr(outbuffer + offset, length_planner_idsi);
      offset += 4;
      memcpy(outbuffer + offset, this->planner_ids[i], length_planner_idsi);
      offset += length_planner_idsi;
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      uint32_t length_name;
      arrToVar(length_name, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_name; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_name-1]=0;
      this->name = (char *)(inbuffer + offset-1);
      offset += length_name;
      uint32_t length_pipeline_id;
      arrToVar(length_pipeline_id, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_pipeline_id; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_pipeline_id-1]=0;
      this->pipeline_id = (char *)(inbuffer + offset-1);
      offset += length_pipeline_id;
      uint32_t planner_ids_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      planner_ids_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      planner_ids_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      planner_ids_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->planner_ids_length);
      if(planner_ids_lengthT > planner_ids_length)
        this->planner_ids = (char**)realloc(this->planner_ids, planner_ids_lengthT * sizeof(char*));
      planner_ids_length = planner_ids_lengthT;
      for( uint32_t i = 0; i < planner_ids_length; i++){
      uint32_t length_st_planner_ids;
      arrToVar(length_st_planner_ids, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_st_planner_ids; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_st_planner_ids-1]=0;
      this->st_planner_ids = (char *)(inbuffer + offset-1);
      offset += length_st_planner_ids;
        memcpy( &(this->planner_ids[i]), &(this->st_planner_ids), sizeof(char*));
      }
     return offset;
    }

    virtual const char * getType() override { return "moveit_msgs/PlannerInterfaceDescription"; };
    virtual const char * getMD5() override { return "3b93afb00ba165a83730c4eb03cd1ab7"; };

  };

}
#endif
