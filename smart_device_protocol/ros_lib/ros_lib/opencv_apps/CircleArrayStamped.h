#ifndef _ROS_opencv_apps_CircleArrayStamped_h
#define _ROS_opencv_apps_CircleArrayStamped_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "opencv_apps/Circle.h"

namespace opencv_apps
{

  class CircleArrayStamped : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      uint32_t circles_length;
      typedef opencv_apps::Circle _circles_type;
      _circles_type st_circles;
      _circles_type * circles;

    CircleArrayStamped():
      header(),
      circles_length(0), st_circles(), circles(nullptr)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
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
      offset += this->header.deserialize(inbuffer + offset);
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

    virtual const char * getType() override { return "opencv_apps/CircleArrayStamped"; };
    virtual const char * getMD5() override { return "430ffa6c2b0a36b7e81feff1ce79c3c4"; };

  };

}
#endif
