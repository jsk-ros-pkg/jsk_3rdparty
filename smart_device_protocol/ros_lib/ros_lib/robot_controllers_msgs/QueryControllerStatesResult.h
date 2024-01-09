#ifndef _ROS_robot_controllers_msgs_QueryControllerStatesResult_h
#define _ROS_robot_controllers_msgs_QueryControllerStatesResult_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "robot_controllers_msgs/ControllerState.h"

namespace robot_controllers_msgs
{

  class QueryControllerStatesResult : public ros::Msg
  {
    public:
      uint32_t state_length;
      typedef robot_controllers_msgs::ControllerState _state_type;
      _state_type st_state;
      _state_type * state;

    QueryControllerStatesResult():
      state_length(0), st_state(), state(nullptr)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->state_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->state_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->state_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->state_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->state_length);
      for( uint32_t i = 0; i < state_length; i++){
      offset += this->state[i].serialize(outbuffer + offset);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      uint32_t state_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      state_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      state_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      state_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->state_length);
      if(state_lengthT > state_length)
        this->state = (robot_controllers_msgs::ControllerState*)realloc(this->state, state_lengthT * sizeof(robot_controllers_msgs::ControllerState));
      state_length = state_lengthT;
      for( uint32_t i = 0; i < state_length; i++){
      offset += this->st_state.deserialize(inbuffer + offset);
        memcpy( &(this->state[i]), &(this->st_state), sizeof(robot_controllers_msgs::ControllerState));
      }
     return offset;
    }

    virtual const char * getType() override { return "robot_controllers_msgs/QueryControllerStatesResult"; };
    virtual const char * getMD5() override { return "95b3426d59527deffe501158443b26c9"; };

  };

}
#endif
