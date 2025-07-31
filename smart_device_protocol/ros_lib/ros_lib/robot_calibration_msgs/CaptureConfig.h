#ifndef _ROS_robot_calibration_msgs_CaptureConfig_h
#define _ROS_robot_calibration_msgs_CaptureConfig_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "sensor_msgs/JointState.h"

namespace robot_calibration_msgs
{

  class CaptureConfig : public ros::Msg
  {
    public:
      typedef sensor_msgs::JointState _joint_states_type;
      _joint_states_type joint_states;
      uint32_t features_length;
      typedef char* _features_type;
      _features_type st_features;
      _features_type * features;

    CaptureConfig():
      joint_states(),
      features_length(0), st_features(), features(nullptr)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->joint_states.serialize(outbuffer + offset);
      *(outbuffer + offset + 0) = (this->features_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->features_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->features_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->features_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->features_length);
      for( uint32_t i = 0; i < features_length; i++){
      uint32_t length_featuresi = strlen(this->features[i]);
      varToArr(outbuffer + offset, length_featuresi);
      offset += 4;
      memcpy(outbuffer + offset, this->features[i], length_featuresi);
      offset += length_featuresi;
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += this->joint_states.deserialize(inbuffer + offset);
      uint32_t features_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      features_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      features_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      features_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->features_length);
      if(features_lengthT > features_length)
        this->features = (char**)realloc(this->features, features_lengthT * sizeof(char*));
      features_length = features_lengthT;
      for( uint32_t i = 0; i < features_length; i++){
      uint32_t length_st_features;
      arrToVar(length_st_features, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_st_features; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_st_features-1]=0;
      this->st_features = (char *)(inbuffer + offset-1);
      offset += length_st_features;
        memcpy( &(this->features[i]), &(this->st_features), sizeof(char*));
      }
     return offset;
    }

    virtual const char * getType() override { return "robot_calibration_msgs/CaptureConfig"; };
    virtual const char * getMD5() override { return "f347b595aa5cb3d9820e25d6d41f25cd"; };

  };

}
#endif
