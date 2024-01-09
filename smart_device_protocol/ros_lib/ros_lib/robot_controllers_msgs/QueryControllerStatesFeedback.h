#ifndef _ROS_robot_controllers_msgs_QueryControllerStatesFeedback_h
#define _ROS_robot_controllers_msgs_QueryControllerStatesFeedback_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "robot_controllers_msgs/ControllerState.h"

namespace robot_controllers_msgs
{

  class QueryControllerStatesFeedback : public ros::Msg
  {
    public:
      uint32_t state_diff_length;
      typedef robot_controllers_msgs::ControllerState _state_diff_type;
      _state_diff_type st_state_diff;
      _state_diff_type * state_diff;

    QueryControllerStatesFeedback():
      state_diff_length(0), st_state_diff(), state_diff(nullptr)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->state_diff_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->state_diff_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->state_diff_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->state_diff_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->state_diff_length);
      for( uint32_t i = 0; i < state_diff_length; i++){
      offset += this->state_diff[i].serialize(outbuffer + offset);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      uint32_t state_diff_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      state_diff_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      state_diff_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      state_diff_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->state_diff_length);
      if(state_diff_lengthT > state_diff_length)
        this->state_diff = (robot_controllers_msgs::ControllerState*)realloc(this->state_diff, state_diff_lengthT * sizeof(robot_controllers_msgs::ControllerState));
      state_diff_length = state_diff_lengthT;
      for( uint32_t i = 0; i < state_diff_length; i++){
      offset += this->st_state_diff.deserialize(inbuffer + offset);
        memcpy( &(this->state_diff[i]), &(this->st_state_diff), sizeof(robot_controllers_msgs::ControllerState));
      }
     return offset;
    }

    virtual const char * getType() override { return "robot_controllers_msgs/QueryControllerStatesFeedback"; };
    virtual const char * getMD5() override { return "884ebbcebb33b2855209539dadfa0dc6"; };

  };

}
#endif
