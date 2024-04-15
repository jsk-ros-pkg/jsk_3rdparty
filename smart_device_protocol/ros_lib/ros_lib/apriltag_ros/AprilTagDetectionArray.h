#ifndef _ROS_apriltag_ros_AprilTagDetectionArray_h
#define _ROS_apriltag_ros_AprilTagDetectionArray_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "apriltag_ros/AprilTagDetection.h"

namespace apriltag_ros
{

  class AprilTagDetectionArray : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      uint32_t detections_length;
      typedef apriltag_ros::AprilTagDetection _detections_type;
      _detections_type st_detections;
      _detections_type * detections;

    AprilTagDetectionArray():
      header(),
      detections_length(0), st_detections(), detections(nullptr)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      *(outbuffer + offset + 0) = (this->detections_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->detections_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->detections_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->detections_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->detections_length);
      for( uint32_t i = 0; i < detections_length; i++){
      offset += this->detections[i].serialize(outbuffer + offset);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      uint32_t detections_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      detections_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      detections_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      detections_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->detections_length);
      if(detections_lengthT > detections_length)
        this->detections = (apriltag_ros::AprilTagDetection*)realloc(this->detections, detections_lengthT * sizeof(apriltag_ros::AprilTagDetection));
      detections_length = detections_lengthT;
      for( uint32_t i = 0; i < detections_length; i++){
      offset += this->st_detections.deserialize(inbuffer + offset);
        memcpy( &(this->detections[i]), &(this->st_detections), sizeof(apriltag_ros::AprilTagDetection));
      }
     return offset;
    }

    virtual const char * getType() override { return "apriltag_ros/AprilTagDetectionArray"; };
    virtual const char * getMD5() override { return "2b6c03434883a5c9897c13b5594dbd91"; };

  };

}
#endif
