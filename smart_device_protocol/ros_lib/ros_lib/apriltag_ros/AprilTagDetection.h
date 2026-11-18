#ifndef _ROS_apriltag_ros_AprilTagDetection_h
#define _ROS_apriltag_ros_AprilTagDetection_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"

namespace apriltag_ros
{

  class AprilTagDetection : public ros::Msg
  {
    public:
      uint32_t id_length;
      typedef int32_t _id_type;
      _id_type st_id;
      _id_type * id;
      uint32_t size_length;
      typedef float _size_type;
      _size_type st_size;
      _size_type * size;
      typedef geometry_msgs::PoseWithCovarianceStamped _pose_type;
      _pose_type pose;

    AprilTagDetection():
      id_length(0), st_id(), id(nullptr),
      size_length(0), st_size(), size(nullptr),
      pose()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->id_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->id_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->id_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->id_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->id_length);
      for( uint32_t i = 0; i < id_length; i++){
      union {
        int32_t real;
        uint32_t base;
      } u_idi;
      u_idi.real = this->id[i];
      *(outbuffer + offset + 0) = (u_idi.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_idi.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_idi.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_idi.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->id[i]);
      }
      *(outbuffer + offset + 0) = (this->size_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->size_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->size_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->size_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->size_length);
      for( uint32_t i = 0; i < size_length; i++){
      offset += serializeAvrFloat64(outbuffer + offset, this->size[i]);
      }
      offset += this->pose.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      uint32_t id_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      id_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      id_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      id_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->id_length);
      if(id_lengthT > id_length)
        this->id = (int32_t*)realloc(this->id, id_lengthT * sizeof(int32_t));
      id_length = id_lengthT;
      for( uint32_t i = 0; i < id_length; i++){
      union {
        int32_t real;
        uint32_t base;
      } u_st_id;
      u_st_id.base = 0;
      u_st_id.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_id.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_st_id.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_st_id.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->st_id = u_st_id.real;
      offset += sizeof(this->st_id);
        memcpy( &(this->id[i]), &(this->st_id), sizeof(int32_t));
      }
      uint32_t size_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      size_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      size_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      size_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->size_length);
      if(size_lengthT > size_length)
        this->size = (float*)realloc(this->size, size_lengthT * sizeof(float));
      size_length = size_lengthT;
      for( uint32_t i = 0; i < size_length; i++){
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->st_size));
        memcpy( &(this->size[i]), &(this->st_size), sizeof(float));
      }
      offset += this->pose.deserialize(inbuffer + offset);
     return offset;
    }

    virtual const char * getType() override { return "apriltag_ros/AprilTagDetection"; };
    virtual const char * getMD5() override { return "090173a6e2b6c8fd96ce000fe9378b4e"; };

  };

}
#endif
