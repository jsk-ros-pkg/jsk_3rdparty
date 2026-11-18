#ifndef _ROS_dialogflow_task_executive_DialogTextResult_h
#define _ROS_dialogflow_task_executive_DialogTextResult_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "dialogflow_task_executive/DialogResponse.h"

namespace dialogflow_task_executive
{

  class DialogTextResult : public ros::Msg
  {
    public:
      typedef dialogflow_task_executive::DialogResponse _response_type;
      _response_type response;
      typedef const char* _session_type;
      _session_type session;
      typedef bool _done_type;
      _done_type done;

    DialogTextResult():
      response(),
      session(""),
      done(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->response.serialize(outbuffer + offset);
      uint32_t length_session = strlen(this->session);
      varToArr(outbuffer + offset, length_session);
      offset += 4;
      memcpy(outbuffer + offset, this->session, length_session);
      offset += length_session;
      union {
        bool real;
        uint8_t base;
      } u_done;
      u_done.real = this->done;
      *(outbuffer + offset + 0) = (u_done.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->done);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += this->response.deserialize(inbuffer + offset);
      uint32_t length_session;
      arrToVar(length_session, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_session; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_session-1]=0;
      this->session = (char *)(inbuffer + offset-1);
      offset += length_session;
      union {
        bool real;
        uint8_t base;
      } u_done;
      u_done.base = 0;
      u_done.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->done = u_done.real;
      offset += sizeof(this->done);
     return offset;
    }

    virtual const char * getType() override { return "dialogflow_task_executive/DialogTextResult"; };
    virtual const char * getMD5() override { return "50694725a4e7e905ed7a64bf3000dffe"; };

  };

}
#endif
