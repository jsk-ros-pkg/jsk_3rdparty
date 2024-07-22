#ifndef _ROS_robot_calibration_msgs_ExtendedCameraInfo_h
#define _ROS_robot_calibration_msgs_ExtendedCameraInfo_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "sensor_msgs/CameraInfo.h"
#include "robot_calibration_msgs/CameraParameter.h"

namespace robot_calibration_msgs
{

  class ExtendedCameraInfo : public ros::Msg
  {
    public:
      typedef sensor_msgs::CameraInfo _camera_info_type;
      _camera_info_type camera_info;
      uint32_t parameters_length;
      typedef robot_calibration_msgs::CameraParameter _parameters_type;
      _parameters_type st_parameters;
      _parameters_type * parameters;

    ExtendedCameraInfo():
      camera_info(),
      parameters_length(0), st_parameters(), parameters(nullptr)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->camera_info.serialize(outbuffer + offset);
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
      offset += this->camera_info.deserialize(inbuffer + offset);
      uint32_t parameters_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      parameters_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      parameters_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      parameters_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->parameters_length);
      if(parameters_lengthT > parameters_length)
        this->parameters = (robot_calibration_msgs::CameraParameter*)realloc(this->parameters, parameters_lengthT * sizeof(robot_calibration_msgs::CameraParameter));
      parameters_length = parameters_lengthT;
      for( uint32_t i = 0; i < parameters_length; i++){
      offset += this->st_parameters.deserialize(inbuffer + offset);
        memcpy( &(this->parameters[i]), &(this->st_parameters), sizeof(robot_calibration_msgs::CameraParameter));
      }
     return offset;
    }

    virtual const char * getType() override { return "robot_calibration_msgs/ExtendedCameraInfo"; };
    virtual const char * getMD5() override { return "d35c5c887e3d90c7f6f9c5a697f44485"; };

  };

}
#endif
