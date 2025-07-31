#ifndef _ROS_SERVICE_GetBodyROI_h
#define _ROS_SERVICE_GetBodyROI_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "naoqi_bridge_msgs/BodyROI.h"

namespace naoqi_bridge_msgs
{

static const char GETBODYROI[] = "naoqi_bridge_msgs/GetBodyROI";

  class GetBodyROIRequest : public ros::Msg
  {
    public:

    GetBodyROIRequest()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
     return offset;
    }

    virtual const char * getType() override { return GETBODYROI; };
    virtual const char * getMD5() override { return "d41d8cd98f00b204e9800998ecf8427e"; };

  };

  class GetBodyROIResponse : public ros::Msg
  {
    public:
      uint32_t bodies_length;
      typedef naoqi_bridge_msgs::BodyROI _bodies_type;
      _bodies_type st_bodies;
      _bodies_type * bodies;

    GetBodyROIResponse():
      bodies_length(0), st_bodies(), bodies(nullptr)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->bodies_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->bodies_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->bodies_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->bodies_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->bodies_length);
      for( uint32_t i = 0; i < bodies_length; i++){
      offset += this->bodies[i].serialize(outbuffer + offset);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      uint32_t bodies_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      bodies_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      bodies_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      bodies_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->bodies_length);
      if(bodies_lengthT > bodies_length)
        this->bodies = (naoqi_bridge_msgs::BodyROI*)realloc(this->bodies, bodies_lengthT * sizeof(naoqi_bridge_msgs::BodyROI));
      bodies_length = bodies_lengthT;
      for( uint32_t i = 0; i < bodies_length; i++){
      offset += this->st_bodies.deserialize(inbuffer + offset);
        memcpy( &(this->bodies[i]), &(this->st_bodies), sizeof(naoqi_bridge_msgs::BodyROI));
      }
     return offset;
    }

    virtual const char * getType() override { return GETBODYROI; };
    virtual const char * getMD5() override { return "ecc2963facbb989a955948135b6e21fd"; };

  };

  class GetBodyROI {
    public:
    typedef GetBodyROIRequest Request;
    typedef GetBodyROIResponse Response;
  };

}
#endif
