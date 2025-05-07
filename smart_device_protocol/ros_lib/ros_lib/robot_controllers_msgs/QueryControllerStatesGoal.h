#ifndef _ROS_robot_controllers_msgs_QueryControllerStatesGoal_h
#define _ROS_robot_controllers_msgs_QueryControllerStatesGoal_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "robot_controllers_msgs/ControllerState.h"

namespace robot_controllers_msgs
{

  class QueryControllerStatesGoal : public ros::Msg
  {
    public:
      uint32_t updates_length;
      typedef robot_controllers_msgs::ControllerState _updates_type;
      _updates_type st_updates;
      _updates_type * updates;

    QueryControllerStatesGoal():
      updates_length(0), st_updates(), updates(nullptr)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->updates_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->updates_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->updates_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->updates_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->updates_length);
      for( uint32_t i = 0; i < updates_length; i++){
      offset += this->updates[i].serialize(outbuffer + offset);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      uint32_t updates_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      updates_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      updates_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      updates_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->updates_length);
      if(updates_lengthT > updates_length)
        this->updates = (robot_controllers_msgs::ControllerState*)realloc(this->updates, updates_lengthT * sizeof(robot_controllers_msgs::ControllerState));
      updates_length = updates_lengthT;
      for( uint32_t i = 0; i < updates_length; i++){
      offset += this->st_updates.deserialize(inbuffer + offset);
        memcpy( &(this->updates[i]), &(this->st_updates), sizeof(robot_controllers_msgs::ControllerState));
      }
     return offset;
    }

    virtual const char * getType() override { return "robot_controllers_msgs/QueryControllerStatesGoal"; };
    virtual const char * getMD5() override { return "6ecbb837d1e8545d81a831a4c1c4bfcc"; };

  };

}
#endif
