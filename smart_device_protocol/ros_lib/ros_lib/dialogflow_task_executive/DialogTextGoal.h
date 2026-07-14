#ifndef _ROS_dialogflow_task_executive_DialogTextGoal_h
#define _ROS_dialogflow_task_executive_DialogTextGoal_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace dialogflow_task_executive
{

  class DialogTextGoal : public ros::Msg
  {
    public:
      typedef const char* _query_type;
      _query_type query;

    DialogTextGoal():
      query("")
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      uint32_t length_query = strlen(this->query);
      varToArr(outbuffer + offset, length_query);
      offset += 4;
      memcpy(outbuffer + offset, this->query, length_query);
      offset += length_query;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      uint32_t length_query;
      arrToVar(length_query, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_query; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_query-1]=0;
      this->query = (char *)(inbuffer + offset-1);
      offset += length_query;
     return offset;
    }

    virtual const char * getType() override { return "dialogflow_task_executive/DialogTextGoal"; };
    virtual const char * getMD5() override { return "6490a46152f373285fe18f491ed93702"; };

  };

}
#endif
