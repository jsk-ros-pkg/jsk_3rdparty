#ifndef _ROS_dialogflow_task_executive_DialogTextFeedback_h
#define _ROS_dialogflow_task_executive_DialogTextFeedback_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace dialogflow_task_executive
{

  class DialogTextFeedback : public ros::Msg
  {
    public:
      typedef const char* _session_type;
      _session_type session;
      typedef const char* _status_type;
      _status_type status;

    DialogTextFeedback():
      session(""),
      status("")
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      uint32_t length_session = strlen(this->session);
      varToArr(outbuffer + offset, length_session);
      offset += 4;
      memcpy(outbuffer + offset, this->session, length_session);
      offset += length_session;
      uint32_t length_status = strlen(this->status);
      varToArr(outbuffer + offset, length_status);
      offset += 4;
      memcpy(outbuffer + offset, this->status, length_status);
      offset += length_status;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      uint32_t length_session;
      arrToVar(length_session, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_session; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_session-1]=0;
      this->session = (char *)(inbuffer + offset-1);
      offset += length_session;
      uint32_t length_status;
      arrToVar(length_status, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_status; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_status-1]=0;
      this->status = (char *)(inbuffer + offset-1);
      offset += length_status;
     return offset;
    }

    virtual const char * getType() override { return "dialogflow_task_executive/DialogTextFeedback"; };
    virtual const char * getMD5() override { return "18931f63f36e4ad1ea67d6ba7c77dceb"; };

  };

}
#endif
