#ifndef _ROS_robot_calibration_msgs_Observation_h
#define _ROS_robot_calibration_msgs_Observation_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "geometry_msgs/PointStamped.h"
#include "robot_calibration_msgs/ExtendedCameraInfo.h"
#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/Image.h"

namespace robot_calibration_msgs
{

  class Observation : public ros::Msg
  {
    public:
      typedef const char* _sensor_name_type;
      _sensor_name_type sensor_name;
      uint32_t features_length;
      typedef geometry_msgs::PointStamped _features_type;
      _features_type st_features;
      _features_type * features;
      typedef robot_calibration_msgs::ExtendedCameraInfo _ext_camera_info_type;
      _ext_camera_info_type ext_camera_info;
      typedef sensor_msgs::PointCloud2 _cloud_type;
      _cloud_type cloud;
      typedef sensor_msgs::Image _image_type;
      _image_type image;

    Observation():
      sensor_name(""),
      features_length(0), st_features(), features(nullptr),
      ext_camera_info(),
      cloud(),
      image()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      uint32_t length_sensor_name = strlen(this->sensor_name);
      varToArr(outbuffer + offset, length_sensor_name);
      offset += 4;
      memcpy(outbuffer + offset, this->sensor_name, length_sensor_name);
      offset += length_sensor_name;
      *(outbuffer + offset + 0) = (this->features_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->features_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->features_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->features_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->features_length);
      for( uint32_t i = 0; i < features_length; i++){
      offset += this->features[i].serialize(outbuffer + offset);
      }
      offset += this->ext_camera_info.serialize(outbuffer + offset);
      offset += this->cloud.serialize(outbuffer + offset);
      offset += this->image.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      uint32_t length_sensor_name;
      arrToVar(length_sensor_name, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_sensor_name; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_sensor_name-1]=0;
      this->sensor_name = (char *)(inbuffer + offset-1);
      offset += length_sensor_name;
      uint32_t features_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      features_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      features_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      features_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->features_length);
      if(features_lengthT > features_length)
        this->features = (geometry_msgs::PointStamped*)realloc(this->features, features_lengthT * sizeof(geometry_msgs::PointStamped));
      features_length = features_lengthT;
      for( uint32_t i = 0; i < features_length; i++){
      offset += this->st_features.deserialize(inbuffer + offset);
        memcpy( &(this->features[i]), &(this->st_features), sizeof(geometry_msgs::PointStamped));
      }
      offset += this->ext_camera_info.deserialize(inbuffer + offset);
      offset += this->cloud.deserialize(inbuffer + offset);
      offset += this->image.deserialize(inbuffer + offset);
     return offset;
    }

    virtual const char * getType() override { return "robot_calibration_msgs/Observation"; };
    virtual const char * getMD5() override { return "b5e5b7c2eb5f83de33806b676db440c9"; };

  };

}
#endif
