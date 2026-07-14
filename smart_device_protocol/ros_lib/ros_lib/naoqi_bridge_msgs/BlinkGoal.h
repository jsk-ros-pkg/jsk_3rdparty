#ifndef _ROS_naoqi_bridge_msgs_BlinkGoal_h
#define _ROS_naoqi_bridge_msgs_BlinkGoal_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/ColorRGBA.h"
#include "ros/duration.h"

namespace naoqi_bridge_msgs
{

  class BlinkGoal : public ros::Msg
  {
    public:
      uint32_t colors_length;
      typedef std_msgs::ColorRGBA _colors_type;
      _colors_type st_colors;
      _colors_type * colors;
      typedef std_msgs::ColorRGBA _bg_color_type;
      _bg_color_type bg_color;
      typedef ros::Duration _blink_duration_type;
      _blink_duration_type blink_duration;
      typedef float _blink_rate_mean_type;
      _blink_rate_mean_type blink_rate_mean;
      typedef float _blink_rate_sd_type;
      _blink_rate_sd_type blink_rate_sd;

    BlinkGoal():
      colors_length(0), st_colors(), colors(nullptr),
      bg_color(),
      blink_duration(),
      blink_rate_mean(0),
      blink_rate_sd(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->colors_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->colors_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->colors_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->colors_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->colors_length);
      for( uint32_t i = 0; i < colors_length; i++){
      offset += this->colors[i].serialize(outbuffer + offset);
      }
      offset += this->bg_color.serialize(outbuffer + offset);
      *(outbuffer + offset + 0) = (this->blink_duration.sec >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->blink_duration.sec >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->blink_duration.sec >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->blink_duration.sec >> (8 * 3)) & 0xFF;
      offset += sizeof(this->blink_duration.sec);
      *(outbuffer + offset + 0) = (this->blink_duration.nsec >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->blink_duration.nsec >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->blink_duration.nsec >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->blink_duration.nsec >> (8 * 3)) & 0xFF;
      offset += sizeof(this->blink_duration.nsec);
      union {
        float real;
        uint32_t base;
      } u_blink_rate_mean;
      u_blink_rate_mean.real = this->blink_rate_mean;
      *(outbuffer + offset + 0) = (u_blink_rate_mean.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_blink_rate_mean.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_blink_rate_mean.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_blink_rate_mean.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->blink_rate_mean);
      union {
        float real;
        uint32_t base;
      } u_blink_rate_sd;
      u_blink_rate_sd.real = this->blink_rate_sd;
      *(outbuffer + offset + 0) = (u_blink_rate_sd.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_blink_rate_sd.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_blink_rate_sd.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_blink_rate_sd.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->blink_rate_sd);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      uint32_t colors_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      colors_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      colors_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      colors_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->colors_length);
      if(colors_lengthT > colors_length)
        this->colors = (std_msgs::ColorRGBA*)realloc(this->colors, colors_lengthT * sizeof(std_msgs::ColorRGBA));
      colors_length = colors_lengthT;
      for( uint32_t i = 0; i < colors_length; i++){
      offset += this->st_colors.deserialize(inbuffer + offset);
        memcpy( &(this->colors[i]), &(this->st_colors), sizeof(std_msgs::ColorRGBA));
      }
      offset += this->bg_color.deserialize(inbuffer + offset);
      this->blink_duration.sec =  ((uint32_t) (*(inbuffer + offset)));
      this->blink_duration.sec |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->blink_duration.sec |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->blink_duration.sec |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->blink_duration.sec);
      this->blink_duration.nsec =  ((uint32_t) (*(inbuffer + offset)));
      this->blink_duration.nsec |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->blink_duration.nsec |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->blink_duration.nsec |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->blink_duration.nsec);
      union {
        float real;
        uint32_t base;
      } u_blink_rate_mean;
      u_blink_rate_mean.base = 0;
      u_blink_rate_mean.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_blink_rate_mean.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_blink_rate_mean.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_blink_rate_mean.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->blink_rate_mean = u_blink_rate_mean.real;
      offset += sizeof(this->blink_rate_mean);
      union {
        float real;
        uint32_t base;
      } u_blink_rate_sd;
      u_blink_rate_sd.base = 0;
      u_blink_rate_sd.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_blink_rate_sd.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_blink_rate_sd.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_blink_rate_sd.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->blink_rate_sd = u_blink_rate_sd.real;
      offset += sizeof(this->blink_rate_sd);
     return offset;
    }

    virtual const char * getType() override { return "naoqi_bridge_msgs/BlinkGoal"; };
    virtual const char * getMD5() override { return "5e5d3c2ba9976dc121a0bb6ef7c66d79"; };

  };

}
#endif
