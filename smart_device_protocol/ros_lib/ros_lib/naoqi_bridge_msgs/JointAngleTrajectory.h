#ifndef _ROS_naoqi_bridge_msgs_JointAngleTrajectory_h
#define _ROS_naoqi_bridge_msgs_JointAngleTrajectory_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"

namespace naoqi_bridge_msgs
{

  class JointAngleTrajectory : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      uint32_t joint_names_length;
      typedef char* _joint_names_type;
      _joint_names_type st_joint_names;
      _joint_names_type * joint_names;
      uint32_t joint_angles_length;
      typedef float _joint_angles_type;
      _joint_angles_type st_joint_angles;
      _joint_angles_type * joint_angles;
      uint32_t times_length;
      typedef float _times_type;
      _times_type st_times;
      _times_type * times;
      typedef uint8_t _relative_type;
      _relative_type relative;

    JointAngleTrajectory():
      header(),
      joint_names_length(0), st_joint_names(), joint_names(nullptr),
      joint_angles_length(0), st_joint_angles(), joint_angles(nullptr),
      times_length(0), st_times(), times(nullptr),
      relative(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      *(outbuffer + offset + 0) = (this->joint_names_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->joint_names_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->joint_names_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->joint_names_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->joint_names_length);
      for( uint32_t i = 0; i < joint_names_length; i++){
      uint32_t length_joint_namesi = strlen(this->joint_names[i]);
      varToArr(outbuffer + offset, length_joint_namesi);
      offset += 4;
      memcpy(outbuffer + offset, this->joint_names[i], length_joint_namesi);
      offset += length_joint_namesi;
      }
      *(outbuffer + offset + 0) = (this->joint_angles_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->joint_angles_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->joint_angles_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->joint_angles_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->joint_angles_length);
      for( uint32_t i = 0; i < joint_angles_length; i++){
      union {
        float real;
        uint32_t base;
      } u_joint_anglesi;
      u_joint_anglesi.real = this->joint_angles[i];
      *(outbuffer + offset + 0) = (u_joint_anglesi.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_joint_anglesi.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_joint_anglesi.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_joint_anglesi.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->joint_angles[i]);
      }
      *(outbuffer + offset + 0) = (this->times_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->times_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->times_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->times_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->times_length);
      for( uint32_t i = 0; i < times_length; i++){
      union {
        float real;
        uint32_t base;
      } u_timesi;
      u_timesi.real = this->times[i];
      *(outbuffer + offset + 0) = (u_timesi.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_timesi.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_timesi.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_timesi.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->times[i]);
      }
      *(outbuffer + offset + 0) = (this->relative >> (8 * 0)) & 0xFF;
      offset += sizeof(this->relative);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      uint32_t joint_names_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      joint_names_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      joint_names_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      joint_names_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->joint_names_length);
      if(joint_names_lengthT > joint_names_length)
        this->joint_names = (char**)realloc(this->joint_names, joint_names_lengthT * sizeof(char*));
      joint_names_length = joint_names_lengthT;
      for( uint32_t i = 0; i < joint_names_length; i++){
      uint32_t length_st_joint_names;
      arrToVar(length_st_joint_names, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_st_joint_names; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_st_joint_names-1]=0;
      this->st_joint_names = (char *)(inbuffer + offset-1);
      offset += length_st_joint_names;
        memcpy( &(this->joint_names[i]), &(this->st_joint_names), sizeof(char*));
      }
      uint32_t joint_angles_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      joint_angles_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      joint_angles_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      joint_angles_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->joint_angles_length);
      if(joint_angles_lengthT > joint_angles_length)
        this->joint_angles = (float*)realloc(this->joint_angles, joint_angles_lengthT * sizeof(float));
      joint_angles_length = joint_angles_lengthT;
      for( uint32_t i = 0; i < joint_angles_length; i++){
      union {
        float real;
        uint32_t base;
      } u_st_joint_angles;
      u_st_joint_angles.base = 0;
      u_st_joint_angles.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_joint_angles.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_st_joint_angles.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_st_joint_angles.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->st_joint_angles = u_st_joint_angles.real;
      offset += sizeof(this->st_joint_angles);
        memcpy( &(this->joint_angles[i]), &(this->st_joint_angles), sizeof(float));
      }
      uint32_t times_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      times_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      times_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      times_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->times_length);
      if(times_lengthT > times_length)
        this->times = (float*)realloc(this->times, times_lengthT * sizeof(float));
      times_length = times_lengthT;
      for( uint32_t i = 0; i < times_length; i++){
      union {
        float real;
        uint32_t base;
      } u_st_times;
      u_st_times.base = 0;
      u_st_times.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_times.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_st_times.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_st_times.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->st_times = u_st_times.real;
      offset += sizeof(this->st_times);
        memcpy( &(this->times[i]), &(this->st_times), sizeof(float));
      }
      this->relative =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->relative);
     return offset;
    }

    virtual const char * getType() override { return "naoqi_bridge_msgs/JointAngleTrajectory"; };
    virtual const char * getMD5() override { return "5ec9aa90e61498421ea53db10da7f8dd"; };

  };

}
#endif
