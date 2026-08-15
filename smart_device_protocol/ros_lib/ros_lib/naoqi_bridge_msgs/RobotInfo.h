#ifndef _ROS_naoqi_bridge_msgs_RobotInfo_h
#define _ROS_naoqi_bridge_msgs_RobotInfo_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace naoqi_bridge_msgs
{

  class RobotInfo : public ros::Msg
  {
    public:
      typedef uint8_t _type_type;
      _type_type type;
      typedef const char* _model_type;
      _model_type model;
      typedef const char* _head_version_type;
      _head_version_type head_version;
      typedef const char* _body_version_type;
      _body_version_type body_version;
      typedef const char* _arm_version_type;
      _arm_version_type arm_version;
      typedef bool _has_laser_type;
      _has_laser_type has_laser;
      typedef bool _has_extended_arms_type;
      _has_extended_arms_type has_extended_arms;
      typedef int32_t _number_of_legs_type;
      _number_of_legs_type number_of_legs;
      typedef int32_t _number_of_arms_type;
      _number_of_arms_type number_of_arms;
      typedef int32_t _number_of_hands_type;
      _number_of_hands_type number_of_hands;
      enum { NAO = 0 };
      enum { ROMEO = 1 };
      enum { PEPPER = 2 };

    RobotInfo():
      type(0),
      model(""),
      head_version(""),
      body_version(""),
      arm_version(""),
      has_laser(0),
      has_extended_arms(0),
      number_of_legs(0),
      number_of_arms(0),
      number_of_hands(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->type >> (8 * 0)) & 0xFF;
      offset += sizeof(this->type);
      uint32_t length_model = strlen(this->model);
      varToArr(outbuffer + offset, length_model);
      offset += 4;
      memcpy(outbuffer + offset, this->model, length_model);
      offset += length_model;
      uint32_t length_head_version = strlen(this->head_version);
      varToArr(outbuffer + offset, length_head_version);
      offset += 4;
      memcpy(outbuffer + offset, this->head_version, length_head_version);
      offset += length_head_version;
      uint32_t length_body_version = strlen(this->body_version);
      varToArr(outbuffer + offset, length_body_version);
      offset += 4;
      memcpy(outbuffer + offset, this->body_version, length_body_version);
      offset += length_body_version;
      uint32_t length_arm_version = strlen(this->arm_version);
      varToArr(outbuffer + offset, length_arm_version);
      offset += 4;
      memcpy(outbuffer + offset, this->arm_version, length_arm_version);
      offset += length_arm_version;
      union {
        bool real;
        uint8_t base;
      } u_has_laser;
      u_has_laser.real = this->has_laser;
      *(outbuffer + offset + 0) = (u_has_laser.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->has_laser);
      union {
        bool real;
        uint8_t base;
      } u_has_extended_arms;
      u_has_extended_arms.real = this->has_extended_arms;
      *(outbuffer + offset + 0) = (u_has_extended_arms.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->has_extended_arms);
      union {
        int32_t real;
        uint32_t base;
      } u_number_of_legs;
      u_number_of_legs.real = this->number_of_legs;
      *(outbuffer + offset + 0) = (u_number_of_legs.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_number_of_legs.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_number_of_legs.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_number_of_legs.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->number_of_legs);
      union {
        int32_t real;
        uint32_t base;
      } u_number_of_arms;
      u_number_of_arms.real = this->number_of_arms;
      *(outbuffer + offset + 0) = (u_number_of_arms.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_number_of_arms.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_number_of_arms.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_number_of_arms.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->number_of_arms);
      union {
        int32_t real;
        uint32_t base;
      } u_number_of_hands;
      u_number_of_hands.real = this->number_of_hands;
      *(outbuffer + offset + 0) = (u_number_of_hands.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_number_of_hands.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_number_of_hands.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_number_of_hands.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->number_of_hands);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      this->type =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->type);
      uint32_t length_model;
      arrToVar(length_model, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_model; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_model-1]=0;
      this->model = (char *)(inbuffer + offset-1);
      offset += length_model;
      uint32_t length_head_version;
      arrToVar(length_head_version, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_head_version; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_head_version-1]=0;
      this->head_version = (char *)(inbuffer + offset-1);
      offset += length_head_version;
      uint32_t length_body_version;
      arrToVar(length_body_version, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_body_version; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_body_version-1]=0;
      this->body_version = (char *)(inbuffer + offset-1);
      offset += length_body_version;
      uint32_t length_arm_version;
      arrToVar(length_arm_version, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_arm_version; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_arm_version-1]=0;
      this->arm_version = (char *)(inbuffer + offset-1);
      offset += length_arm_version;
      union {
        bool real;
        uint8_t base;
      } u_has_laser;
      u_has_laser.base = 0;
      u_has_laser.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->has_laser = u_has_laser.real;
      offset += sizeof(this->has_laser);
      union {
        bool real;
        uint8_t base;
      } u_has_extended_arms;
      u_has_extended_arms.base = 0;
      u_has_extended_arms.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->has_extended_arms = u_has_extended_arms.real;
      offset += sizeof(this->has_extended_arms);
      union {
        int32_t real;
        uint32_t base;
      } u_number_of_legs;
      u_number_of_legs.base = 0;
      u_number_of_legs.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_number_of_legs.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_number_of_legs.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_number_of_legs.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->number_of_legs = u_number_of_legs.real;
      offset += sizeof(this->number_of_legs);
      union {
        int32_t real;
        uint32_t base;
      } u_number_of_arms;
      u_number_of_arms.base = 0;
      u_number_of_arms.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_number_of_arms.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_number_of_arms.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_number_of_arms.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->number_of_arms = u_number_of_arms.real;
      offset += sizeof(this->number_of_arms);
      union {
        int32_t real;
        uint32_t base;
      } u_number_of_hands;
      u_number_of_hands.base = 0;
      u_number_of_hands.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_number_of_hands.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_number_of_hands.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_number_of_hands.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->number_of_hands = u_number_of_hands.real;
      offset += sizeof(this->number_of_hands);
     return offset;
    }

    virtual const char * getType() override { return "naoqi_bridge_msgs/RobotInfo"; };
    virtual const char * getMD5() override { return "cc8c56c1600e9f458ce3f2626800e77f"; };

  };

}
#endif
