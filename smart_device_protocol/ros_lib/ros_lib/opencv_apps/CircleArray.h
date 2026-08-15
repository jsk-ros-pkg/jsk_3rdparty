#ifndef _ROS_opencv_apps_CircleArray_h
#define _ROS_opencv_apps_CircleArray_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "opencv_apps/Circle.h"

namespace opencv_apps
{

  class CircleArray : public ros::Msg
  {
    public:
      uint32_t circles_length;
      typedef opencv_apps::Circle _circles_type;
      _circles_type st_circles;
      _circles_type * circles;

    CircleArray():
      circles_length(0), st_circles(), circles(nullptr)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->circles_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->circles_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->circles_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->circles_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->circles_length);
      for( uint32_t i = 0; i < circles_length; i++){
      offset += this->circles[i].serialize(outbuffer + offset);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      uint32_t circles_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      circles_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      circles_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      circles_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->circles_length);
      if(circles_lengthT > circles_length)
        this->circles = (opencv_apps::Circle*)realloc(this->circles, circles_lengthT * sizeof(opencv_apps::Circle));
      circles_length = circles_lengthT;
      for( uint32_t i = 0; i < circles_length; i++){
      offset += this->st_circles.deserialize(inbuffer + offset);
        memcpy( &(this->circles[i]), &(this->st_circles), sizeof(opencv_apps::Circle));
      }
     return offset;
    }

    virtual const char * getType() override { return "opencv_apps/CircleArray"; };
    virtual const char * getMD5() override { return "1970b146e338dd024c765e522039a727"; };

  };

}
#endif
