#ifndef _ROS_naoqi_bridge_msgs_FadeRGB_h
#define _ROS_naoqi_bridge_msgs_FadeRGB_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/ColorRGBA.h"
#include "ros/duration.h"

namespace naoqi_bridge_msgs
{

  class FadeRGB : public ros::Msg
  {
    public:
      typedef const char* _led_name_type;
      _led_name_type led_name;
      typedef std_msgs::ColorRGBA _color_type;
      _color_type color;
      typedef ros::Duration _fade_duration_type;
      _fade_duration_type fade_duration;

    FadeRGB():
      led_name(""),
      color(),
      fade_duration()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      uint32_t length_led_name = strlen(this->led_name);
      varToArr(outbuffer + offset, length_led_name);
      offset += 4;
      memcpy(outbuffer + offset, this->led_name, length_led_name);
      offset += length_led_name;
      offset += this->color.serialize(outbuffer + offset);
      *(outbuffer + offset + 0) = (this->fade_duration.sec >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->fade_duration.sec >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->fade_duration.sec >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->fade_duration.sec >> (8 * 3)) & 0xFF;
      offset += sizeof(this->fade_duration.sec);
      *(outbuffer + offset + 0) = (this->fade_duration.nsec >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->fade_duration.nsec >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->fade_duration.nsec >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->fade_duration.nsec >> (8 * 3)) & 0xFF;
      offset += sizeof(this->fade_duration.nsec);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      uint32_t length_led_name;
      arrToVar(length_led_name, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_led_name; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_led_name-1]=0;
      this->led_name = (char *)(inbuffer + offset-1);
      offset += length_led_name;
      offset += this->color.deserialize(inbuffer + offset);
      this->fade_duration.sec =  ((uint32_t) (*(inbuffer + offset)));
      this->fade_duration.sec |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->fade_duration.sec |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->fade_duration.sec |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->fade_duration.sec);
      this->fade_duration.nsec =  ((uint32_t) (*(inbuffer + offset)));
      this->fade_duration.nsec |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->fade_duration.nsec |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->fade_duration.nsec |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->fade_duration.nsec);
     return offset;
    }

    virtual const char * getType() override { return "naoqi_bridge_msgs/FadeRGB"; };
    virtual const char * getMD5() override { return "0df8c8fbe7f1de5f2168d6117ffced08"; };

  };

}
#endif
