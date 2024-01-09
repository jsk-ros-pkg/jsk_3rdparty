#ifndef _ROS_pr2_mechanism_controllers_Odometer_h
#define _ROS_pr2_mechanism_controllers_Odometer_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace pr2_mechanism_controllers
{

  class Odometer : public ros::Msg
  {
    public:
      typedef float _distance_type;
      _distance_type distance;
      typedef float _angle_type;
      _angle_type angle;

    Odometer():
      distance(0),
      angle(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += serializeAvrFloat64(outbuffer + offset, this->distance);
      offset += serializeAvrFloat64(outbuffer + offset, this->angle);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->distance));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->angle));
     return offset;
    }

    virtual const char * getType() override { return "pr2_mechanism_controllers/Odometer"; };
    virtual const char * getMD5() override { return "1f1d53743f4592ee455aa3eaf9019457"; };

  };

}
#endif
