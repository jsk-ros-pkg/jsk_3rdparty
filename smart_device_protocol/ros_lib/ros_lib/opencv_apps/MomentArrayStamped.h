#ifndef _ROS_opencv_apps_MomentArrayStamped_h
#define _ROS_opencv_apps_MomentArrayStamped_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "opencv_apps/Moment.h"

namespace opencv_apps
{

  class MomentArrayStamped : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      uint32_t moments_length;
      typedef opencv_apps::Moment _moments_type;
      _moments_type st_moments;
      _moments_type * moments;

    MomentArrayStamped():
      header(),
      moments_length(0), st_moments(), moments(nullptr)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      *(outbuffer + offset + 0) = (this->moments_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->moments_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->moments_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->moments_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->moments_length);
      for( uint32_t i = 0; i < moments_length; i++){
      offset += this->moments[i].serialize(outbuffer + offset);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      uint32_t moments_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      moments_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      moments_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      moments_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->moments_length);
      if(moments_lengthT > moments_length)
        this->moments = (opencv_apps::Moment*)realloc(this->moments, moments_lengthT * sizeof(opencv_apps::Moment));
      moments_length = moments_lengthT;
      for( uint32_t i = 0; i < moments_length; i++){
      offset += this->st_moments.deserialize(inbuffer + offset);
        memcpy( &(this->moments[i]), &(this->st_moments), sizeof(opencv_apps::Moment));
      }
     return offset;
    }

    virtual const char * getType() override { return "opencv_apps/MomentArrayStamped"; };
    virtual const char * getMD5() override { return "28ac0beb70b037acf76c3bed71b679a9"; };

  };

}
#endif
