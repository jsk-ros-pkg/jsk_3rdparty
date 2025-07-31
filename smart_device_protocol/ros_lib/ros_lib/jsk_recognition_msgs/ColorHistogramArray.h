#ifndef _ROS_jsk_recognition_msgs_ColorHistogramArray_h
#define _ROS_jsk_recognition_msgs_ColorHistogramArray_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "jsk_recognition_msgs/ColorHistogram.h"

namespace jsk_recognition_msgs
{

  class ColorHistogramArray : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      uint32_t histograms_length;
      typedef jsk_recognition_msgs::ColorHistogram _histograms_type;
      _histograms_type st_histograms;
      _histograms_type * histograms;

    ColorHistogramArray():
      header(),
      histograms_length(0), st_histograms(), histograms(nullptr)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      *(outbuffer + offset + 0) = (this->histograms_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->histograms_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->histograms_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->histograms_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->histograms_length);
      for( uint32_t i = 0; i < histograms_length; i++){
      offset += this->histograms[i].serialize(outbuffer + offset);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      uint32_t histograms_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      histograms_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      histograms_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      histograms_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->histograms_length);
      if(histograms_lengthT > histograms_length)
        this->histograms = (jsk_recognition_msgs::ColorHistogram*)realloc(this->histograms, histograms_lengthT * sizeof(jsk_recognition_msgs::ColorHistogram));
      histograms_length = histograms_lengthT;
      for( uint32_t i = 0; i < histograms_length; i++){
      offset += this->st_histograms.deserialize(inbuffer + offset);
        memcpy( &(this->histograms[i]), &(this->st_histograms), sizeof(jsk_recognition_msgs::ColorHistogram));
      }
     return offset;
    }

    virtual const char * getType() override { return "jsk_recognition_msgs/ColorHistogramArray"; };
    virtual const char * getMD5() override { return "3bcc7f05c5520f311047096d5530e715"; };

  };

}
#endif
