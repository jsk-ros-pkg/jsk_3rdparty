#ifndef _ROS_fetch_driver_msgs_GripperState_h
#define _ROS_fetch_driver_msgs_GripperState_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "fetch_driver_msgs/BoardState.h"
#include "fetch_driver_msgs/MotorState.h"
#include "fetch_driver_msgs/JointState.h"

namespace fetch_driver_msgs
{

  class GripperState : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef bool _ready_type;
      _ready_type ready;
      typedef bool _faulted_type;
      _faulted_type faulted;
      uint32_t boards_length;
      typedef fetch_driver_msgs::BoardState _boards_type;
      _boards_type st_boards;
      _boards_type * boards;
      uint32_t motors_length;
      typedef fetch_driver_msgs::MotorState _motors_type;
      _motors_type st_motors;
      _motors_type * motors;
      uint32_t joints_length;
      typedef fetch_driver_msgs::JointState _joints_type;
      _joints_type st_joints;
      _joints_type * joints;

    GripperState():
      header(),
      ready(0),
      faulted(0),
      boards_length(0), st_boards(), boards(nullptr),
      motors_length(0), st_motors(), motors(nullptr),
      joints_length(0), st_joints(), joints(nullptr)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      union {
        bool real;
        uint8_t base;
      } u_ready;
      u_ready.real = this->ready;
      *(outbuffer + offset + 0) = (u_ready.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->ready);
      union {
        bool real;
        uint8_t base;
      } u_faulted;
      u_faulted.real = this->faulted;
      *(outbuffer + offset + 0) = (u_faulted.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->faulted);
      *(outbuffer + offset + 0) = (this->boards_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->boards_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->boards_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->boards_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->boards_length);
      for( uint32_t i = 0; i < boards_length; i++){
      offset += this->boards[i].serialize(outbuffer + offset);
      }
      *(outbuffer + offset + 0) = (this->motors_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->motors_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->motors_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->motors_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->motors_length);
      for( uint32_t i = 0; i < motors_length; i++){
      offset += this->motors[i].serialize(outbuffer + offset);
      }
      *(outbuffer + offset + 0) = (this->joints_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->joints_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->joints_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->joints_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->joints_length);
      for( uint32_t i = 0; i < joints_length; i++){
      offset += this->joints[i].serialize(outbuffer + offset);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      union {
        bool real;
        uint8_t base;
      } u_ready;
      u_ready.base = 0;
      u_ready.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->ready = u_ready.real;
      offset += sizeof(this->ready);
      union {
        bool real;
        uint8_t base;
      } u_faulted;
      u_faulted.base = 0;
      u_faulted.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->faulted = u_faulted.real;
      offset += sizeof(this->faulted);
      uint32_t boards_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      boards_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      boards_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      boards_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->boards_length);
      if(boards_lengthT > boards_length)
        this->boards = (fetch_driver_msgs::BoardState*)realloc(this->boards, boards_lengthT * sizeof(fetch_driver_msgs::BoardState));
      boards_length = boards_lengthT;
      for( uint32_t i = 0; i < boards_length; i++){
      offset += this->st_boards.deserialize(inbuffer + offset);
        memcpy( &(this->boards[i]), &(this->st_boards), sizeof(fetch_driver_msgs::BoardState));
      }
      uint32_t motors_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      motors_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      motors_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      motors_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->motors_length);
      if(motors_lengthT > motors_length)
        this->motors = (fetch_driver_msgs::MotorState*)realloc(this->motors, motors_lengthT * sizeof(fetch_driver_msgs::MotorState));
      motors_length = motors_lengthT;
      for( uint32_t i = 0; i < motors_length; i++){
      offset += this->st_motors.deserialize(inbuffer + offset);
        memcpy( &(this->motors[i]), &(this->st_motors), sizeof(fetch_driver_msgs::MotorState));
      }
      uint32_t joints_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      joints_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      joints_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      joints_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->joints_length);
      if(joints_lengthT > joints_length)
        this->joints = (fetch_driver_msgs::JointState*)realloc(this->joints, joints_lengthT * sizeof(fetch_driver_msgs::JointState));
      joints_length = joints_lengthT;
      for( uint32_t i = 0; i < joints_length; i++){
      offset += this->st_joints.deserialize(inbuffer + offset);
        memcpy( &(this->joints[i]), &(this->st_joints), sizeof(fetch_driver_msgs::JointState));
      }
     return offset;
    }

    virtual const char * getType() override { return "fetch_driver_msgs/GripperState"; };
    virtual const char * getMD5() override { return "3b442b195034293254bd9964c5dac688"; };

  };

}
#endif
