#ifndef _ROS_pr2_mechanism_controllers_TrackLinkCmd_h
#define _ROS_pr2_mechanism_controllers_TrackLinkCmd_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "geometry_msgs/Point.h"

namespace pr2_mechanism_controllers
{

  class TrackLinkCmd : public ros::Msg
  {
    public:
      typedef int8_t _enable_type;
      _enable_type enable;
      typedef const char* _link_name_type;
      _link_name_type link_name;
      typedef geometry_msgs::Point _point_type;
      _point_type point;

    TrackLinkCmd():
      enable(0),
      link_name(""),
      point()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      union {
        int8_t real;
        uint8_t base;
      } u_enable;
      u_enable.real = this->enable;
      *(outbuffer + offset + 0) = (u_enable.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->enable);
      uint32_t length_link_name = strlen(this->link_name);
      varToArr(outbuffer + offset, length_link_name);
      offset += 4;
      memcpy(outbuffer + offset, this->link_name, length_link_name);
      offset += length_link_name;
      offset += this->point.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      union {
        int8_t real;
        uint8_t base;
      } u_enable;
      u_enable.base = 0;
      u_enable.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->enable = u_enable.real;
      offset += sizeof(this->enable);
      uint32_t length_link_name;
      arrToVar(length_link_name, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_link_name; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_link_name-1]=0;
      this->link_name = (char *)(inbuffer + offset-1);
      offset += length_link_name;
      offset += this->point.deserialize(inbuffer + offset);
     return offset;
    }

    virtual const char * getType() override { return "pr2_mechanism_controllers/TrackLinkCmd"; };
    virtual const char * getMD5() override { return "08ccfe603e4e21c792896712c3b72de2"; };

  };

}
#endif
