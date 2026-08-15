#ifndef _ROS_opencv_apps_ContourArray_h
#define _ROS_opencv_apps_ContourArray_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "opencv_apps/Contour.h"

namespace opencv_apps
{

  class ContourArray : public ros::Msg
  {
    public:
      uint32_t contours_length;
      typedef opencv_apps::Contour _contours_type;
      _contours_type st_contours;
      _contours_type * contours;

    ContourArray():
      contours_length(0), st_contours(), contours(nullptr)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->contours_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->contours_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->contours_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->contours_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->contours_length);
      for( uint32_t i = 0; i < contours_length; i++){
      offset += this->contours[i].serialize(outbuffer + offset);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      uint32_t contours_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      contours_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      contours_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      contours_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->contours_length);
      if(contours_lengthT > contours_length)
        this->contours = (opencv_apps::Contour*)realloc(this->contours, contours_lengthT * sizeof(opencv_apps::Contour));
      contours_length = contours_lengthT;
      for( uint32_t i = 0; i < contours_length; i++){
      offset += this->st_contours.deserialize(inbuffer + offset);
        memcpy( &(this->contours[i]), &(this->st_contours), sizeof(opencv_apps::Contour));
      }
     return offset;
    }

    virtual const char * getType() override { return "opencv_apps/ContourArray"; };
    virtual const char * getMD5() override { return "fc54374f45559dfed248b316ccf9181d"; };

  };

}
#endif
