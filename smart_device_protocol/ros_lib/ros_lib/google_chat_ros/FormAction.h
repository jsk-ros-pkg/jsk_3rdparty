#ifndef _ROS_google_chat_ros_FormAction_h
#define _ROS_google_chat_ros_FormAction_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "google_chat_ros/ActionParameter.h"

namespace google_chat_ros
{

  class FormAction : public ros::Msg
  {
    public:
      typedef const char* _action_method_name_type;
      _action_method_name_type action_method_name;
      uint32_t parameters_length;
      typedef google_chat_ros::ActionParameter _parameters_type;
      _parameters_type st_parameters;
      _parameters_type * parameters;

    FormAction():
      action_method_name(""),
      parameters_length(0), st_parameters(), parameters(nullptr)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      uint32_t length_action_method_name = strlen(this->action_method_name);
      varToArr(outbuffer + offset, length_action_method_name);
      offset += 4;
      memcpy(outbuffer + offset, this->action_method_name, length_action_method_name);
      offset += length_action_method_name;
      *(outbuffer + offset + 0) = (this->parameters_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->parameters_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->parameters_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->parameters_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->parameters_length);
      for( uint32_t i = 0; i < parameters_length; i++){
      offset += this->parameters[i].serialize(outbuffer + offset);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      uint32_t length_action_method_name;
      arrToVar(length_action_method_name, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_action_method_name; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_action_method_name-1]=0;
      this->action_method_name = (char *)(inbuffer + offset-1);
      offset += length_action_method_name;
      uint32_t parameters_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      parameters_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      parameters_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      parameters_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->parameters_length);
      if(parameters_lengthT > parameters_length)
        this->parameters = (google_chat_ros::ActionParameter*)realloc(this->parameters, parameters_lengthT * sizeof(google_chat_ros::ActionParameter));
      parameters_length = parameters_lengthT;
      for( uint32_t i = 0; i < parameters_length; i++){
      offset += this->st_parameters.deserialize(inbuffer + offset);
        memcpy( &(this->parameters[i]), &(this->st_parameters), sizeof(google_chat_ros::ActionParameter));
      }
     return offset;
    }

    virtual const char * getType() override { return "google_chat_ros/FormAction"; };
    virtual const char * getMD5() override { return "1b529563550f4567d19b052af042c195"; };

  };

}
#endif
