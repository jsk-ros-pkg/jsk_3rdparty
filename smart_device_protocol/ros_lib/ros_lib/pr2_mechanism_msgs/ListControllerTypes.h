#ifndef _ROS_SERVICE_ListControllerTypes_h
#define _ROS_SERVICE_ListControllerTypes_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace pr2_mechanism_msgs
{

static const char LISTCONTROLLERTYPES[] = "pr2_mechanism_msgs/ListControllerTypes";

  class ListControllerTypesRequest : public ros::Msg
  {
    public:

    ListControllerTypesRequest()
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

    virtual const char * getType() override { return LISTCONTROLLERTYPES; };
    virtual const char * getMD5() override { return "d41d8cd98f00b204e9800998ecf8427e"; };

  };

  class ListControllerTypesResponse : public ros::Msg
  {
    public:
      uint32_t types_length;
      typedef char* _types_type;
      _types_type st_types;
      _types_type * types;

    ListControllerTypesResponse():
      types_length(0), st_types(), types(nullptr)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->types_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->types_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->types_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->types_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->types_length);
      for( uint32_t i = 0; i < types_length; i++){
      uint32_t length_typesi = strlen(this->types[i]);
      varToArr(outbuffer + offset, length_typesi);
      offset += 4;
      memcpy(outbuffer + offset, this->types[i], length_typesi);
      offset += length_typesi;
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      uint32_t types_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      types_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      types_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      types_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->types_length);
      if(types_lengthT > types_length)
        this->types = (char**)realloc(this->types, types_lengthT * sizeof(char*));
      types_length = types_lengthT;
      for( uint32_t i = 0; i < types_length; i++){
      uint32_t length_st_types;
      arrToVar(length_st_types, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_st_types; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_st_types-1]=0;
      this->st_types = (char *)(inbuffer + offset-1);
      offset += length_st_types;
        memcpy( &(this->types[i]), &(this->st_types), sizeof(char*));
      }
     return offset;
    }

    virtual const char * getType() override { return LISTCONTROLLERTYPES; };
    virtual const char * getMD5() override { return "80aee506387f88339842e9a93044c959"; };

  };

  class ListControllerTypes {
    public:
    typedef ListControllerTypesRequest Request;
    typedef ListControllerTypesResponse Response;
  };

}
#endif
