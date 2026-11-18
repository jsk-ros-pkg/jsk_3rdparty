#ifndef _ROS_SERVICE_AnalyzeSingleImage_h
#define _ROS_SERVICE_AnalyzeSingleImage_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "apriltag_ros/AprilTagDetectionArray.h"
#include "sensor_msgs/CameraInfo.h"

namespace apriltag_ros
{

static const char ANALYZESINGLEIMAGE[] = "apriltag_ros/AnalyzeSingleImage";

  class AnalyzeSingleImageRequest : public ros::Msg
  {
    public:
      typedef const char* _full_path_where_to_get_image_type;
      _full_path_where_to_get_image_type full_path_where_to_get_image;
      typedef const char* _full_path_where_to_save_image_type;
      _full_path_where_to_save_image_type full_path_where_to_save_image;
      typedef sensor_msgs::CameraInfo _camera_info_type;
      _camera_info_type camera_info;

    AnalyzeSingleImageRequest():
      full_path_where_to_get_image(""),
      full_path_where_to_save_image(""),
      camera_info()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      uint32_t length_full_path_where_to_get_image = strlen(this->full_path_where_to_get_image);
      varToArr(outbuffer + offset, length_full_path_where_to_get_image);
      offset += 4;
      memcpy(outbuffer + offset, this->full_path_where_to_get_image, length_full_path_where_to_get_image);
      offset += length_full_path_where_to_get_image;
      uint32_t length_full_path_where_to_save_image = strlen(this->full_path_where_to_save_image);
      varToArr(outbuffer + offset, length_full_path_where_to_save_image);
      offset += 4;
      memcpy(outbuffer + offset, this->full_path_where_to_save_image, length_full_path_where_to_save_image);
      offset += length_full_path_where_to_save_image;
      offset += this->camera_info.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      uint32_t length_full_path_where_to_get_image;
      arrToVar(length_full_path_where_to_get_image, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_full_path_where_to_get_image; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_full_path_where_to_get_image-1]=0;
      this->full_path_where_to_get_image = (char *)(inbuffer + offset-1);
      offset += length_full_path_where_to_get_image;
      uint32_t length_full_path_where_to_save_image;
      arrToVar(length_full_path_where_to_save_image, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_full_path_where_to_save_image; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_full_path_where_to_save_image-1]=0;
      this->full_path_where_to_save_image = (char *)(inbuffer + offset-1);
      offset += length_full_path_where_to_save_image;
      offset += this->camera_info.deserialize(inbuffer + offset);
     return offset;
    }

    virtual const char * getType() override { return ANALYZESINGLEIMAGE; };
    virtual const char * getMD5() override { return "ce260db7e8fcb58cbea397e93c5438a4"; };

  };

  class AnalyzeSingleImageResponse : public ros::Msg
  {
    public:
      typedef apriltag_ros::AprilTagDetectionArray _tag_detections_type;
      _tag_detections_type tag_detections;

    AnalyzeSingleImageResponse():
      tag_detections()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->tag_detections.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += this->tag_detections.deserialize(inbuffer + offset);
     return offset;
    }

    virtual const char * getType() override { return ANALYZESINGLEIMAGE; };
    virtual const char * getMD5() override { return "252b618af4df2baf843a5edd035f3c2c"; };

  };

  class AnalyzeSingleImage {
    public:
    typedef AnalyzeSingleImageRequest Request;
    typedef AnalyzeSingleImageResponse Response;
  };

}
#endif
