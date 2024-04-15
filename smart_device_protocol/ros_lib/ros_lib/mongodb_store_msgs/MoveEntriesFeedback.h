#ifndef _ROS_mongodb_store_msgs_MoveEntriesFeedback_h
#define _ROS_mongodb_store_msgs_MoveEntriesFeedback_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace mongodb_store_msgs
{

  class MoveEntriesFeedback : public ros::Msg
  {
    public:
      uint32_t completed_length;
      typedef char* _completed_type;
      _completed_type st_completed;
      _completed_type * completed;

    MoveEntriesFeedback():
      completed_length(0), st_completed(), completed(nullptr)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->completed_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->completed_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->completed_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->completed_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->completed_length);
      for( uint32_t i = 0; i < completed_length; i++){
      uint32_t length_completedi = strlen(this->completed[i]);
      varToArr(outbuffer + offset, length_completedi);
      offset += 4;
      memcpy(outbuffer + offset, this->completed[i], length_completedi);
      offset += length_completedi;
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      uint32_t completed_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      completed_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      completed_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      completed_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->completed_length);
      if(completed_lengthT > completed_length)
        this->completed = (char**)realloc(this->completed, completed_lengthT * sizeof(char*));
      completed_length = completed_lengthT;
      for( uint32_t i = 0; i < completed_length; i++){
      uint32_t length_st_completed;
      arrToVar(length_st_completed, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_st_completed; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_st_completed-1]=0;
      this->st_completed = (char *)(inbuffer + offset-1);
      offset += length_st_completed;
        memcpy( &(this->completed[i]), &(this->st_completed), sizeof(char*));
      }
     return offset;
    }

    virtual const char * getType() override { return "mongodb_store_msgs/MoveEntriesFeedback"; };
    virtual const char * getMD5() override { return "a62bad29223cd7da9d6f04397aee5b94"; };

  };

}
#endif
