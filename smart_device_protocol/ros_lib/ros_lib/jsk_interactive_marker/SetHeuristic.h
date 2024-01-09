#ifndef _ROS_SERVICE_SetHeuristic_h
#define _ROS_SERVICE_SetHeuristic_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace jsk_interactive_marker
{

static const char SETHEURISTIC[] = "jsk_interactive_marker/SetHeuristic";

  class SetHeuristicRequest : public ros::Msg
  {
    public:
      typedef const char* _heuristic_type;
      _heuristic_type heuristic;

    SetHeuristicRequest():
      heuristic("")
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      uint32_t length_heuristic = strlen(this->heuristic);
      varToArr(outbuffer + offset, length_heuristic);
      offset += 4;
      memcpy(outbuffer + offset, this->heuristic, length_heuristic);
      offset += length_heuristic;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      uint32_t length_heuristic;
      arrToVar(length_heuristic, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_heuristic; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_heuristic-1]=0;
      this->heuristic = (char *)(inbuffer + offset-1);
      offset += length_heuristic;
     return offset;
    }

    virtual const char * getType() override { return SETHEURISTIC; };
    virtual const char * getMD5() override { return "96bf1327fac533122d937324246cbde4"; };

  };

  class SetHeuristicResponse : public ros::Msg
  {
    public:

    SetHeuristicResponse()
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

    virtual const char * getType() override { return SETHEURISTIC; };
    virtual const char * getMD5() override { return "d41d8cd98f00b204e9800998ecf8427e"; };

  };

  class SetHeuristic {
    public:
    typedef SetHeuristicRequest Request;
    typedef SetHeuristicResponse Response;
  };

}
#endif
