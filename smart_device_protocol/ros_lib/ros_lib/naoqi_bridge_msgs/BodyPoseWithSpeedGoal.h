#ifndef _ROS_naoqi_bridge_msgs_BodyPoseWithSpeedGoal_h
#define _ROS_naoqi_bridge_msgs_BodyPoseWithSpeedGoal_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace naoqi_bridge_msgs
{

  class BodyPoseWithSpeedGoal : public ros::Msg
  {
    public:
      typedef const char* _posture_name_type;
      _posture_name_type posture_name;
      typedef float _speed_type;
      _speed_type speed;

    BodyPoseWithSpeedGoal():
      posture_name(""),
      speed(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      uint32_t length_posture_name = strlen(this->posture_name);
      varToArr(outbuffer + offset, length_posture_name);
      offset += 4;
      memcpy(outbuffer + offset, this->posture_name, length_posture_name);
      offset += length_posture_name;
      union {
        float real;
        uint32_t base;
      } u_speed;
      u_speed.real = this->speed;
      *(outbuffer + offset + 0) = (u_speed.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_speed.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_speed.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_speed.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->speed);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      uint32_t length_posture_name;
      arrToVar(length_posture_name, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_posture_name; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_posture_name-1]=0;
      this->posture_name = (char *)(inbuffer + offset-1);
      offset += length_posture_name;
      union {
        float real;
        uint32_t base;
      } u_speed;
      u_speed.base = 0;
      u_speed.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_speed.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_speed.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_speed.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->speed = u_speed.real;
      offset += sizeof(this->speed);
     return offset;
    }

    virtual const char * getType() override { return "naoqi_bridge_msgs/BodyPoseWithSpeedGoal"; };
    virtual const char * getMD5() override { return "6c5f7bd37d2a5befe00383fa440a8f6e"; };

  };

}
#endif
