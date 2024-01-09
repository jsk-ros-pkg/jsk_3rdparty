#ifndef _ROS_SERVICE_GetInstalledBehaviors_h
#define _ROS_SERVICE_GetInstalledBehaviors_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace naoqi_bridge_msgs
{

static const char GETINSTALLEDBEHAVIORS[] = "naoqi_bridge_msgs/GetInstalledBehaviors";

  class GetInstalledBehaviorsRequest : public ros::Msg
  {
    public:

    GetInstalledBehaviorsRequest()
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

    virtual const char * getType() override { return GETINSTALLEDBEHAVIORS; };
    virtual const char * getMD5() override { return "d41d8cd98f00b204e9800998ecf8427e"; };

  };

  class GetInstalledBehaviorsResponse : public ros::Msg
  {
    public:
      uint32_t behaviors_length;
      typedef char* _behaviors_type;
      _behaviors_type st_behaviors;
      _behaviors_type * behaviors;

    GetInstalledBehaviorsResponse():
      behaviors_length(0), st_behaviors(), behaviors(nullptr)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->behaviors_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->behaviors_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->behaviors_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->behaviors_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->behaviors_length);
      for( uint32_t i = 0; i < behaviors_length; i++){
      uint32_t length_behaviorsi = strlen(this->behaviors[i]);
      varToArr(outbuffer + offset, length_behaviorsi);
      offset += 4;
      memcpy(outbuffer + offset, this->behaviors[i], length_behaviorsi);
      offset += length_behaviorsi;
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      uint32_t behaviors_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      behaviors_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      behaviors_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      behaviors_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->behaviors_length);
      if(behaviors_lengthT > behaviors_length)
        this->behaviors = (char**)realloc(this->behaviors, behaviors_lengthT * sizeof(char*));
      behaviors_length = behaviors_lengthT;
      for( uint32_t i = 0; i < behaviors_length; i++){
      uint32_t length_st_behaviors;
      arrToVar(length_st_behaviors, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_st_behaviors; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_st_behaviors-1]=0;
      this->st_behaviors = (char *)(inbuffer + offset-1);
      offset += length_st_behaviors;
        memcpy( &(this->behaviors[i]), &(this->st_behaviors), sizeof(char*));
      }
     return offset;
    }

    virtual const char * getType() override { return GETINSTALLEDBEHAVIORS; };
    virtual const char * getMD5() override { return "715783c8c6eb28fc2e1c05184add75ec"; };

  };

  class GetInstalledBehaviors {
    public:
    typedef GetInstalledBehaviorsRequest Request;
    typedef GetInstalledBehaviorsResponse Response;
  };

}
#endif
