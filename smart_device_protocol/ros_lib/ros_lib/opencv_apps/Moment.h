#ifndef _ROS_opencv_apps_Moment_h
#define _ROS_opencv_apps_Moment_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "opencv_apps/Point2D.h"

namespace opencv_apps
{

  class Moment : public ros::Msg
  {
    public:
      typedef float _m00_type;
      _m00_type m00;
      typedef float _m10_type;
      _m10_type m10;
      typedef float _m01_type;
      _m01_type m01;
      typedef float _m20_type;
      _m20_type m20;
      typedef float _m11_type;
      _m11_type m11;
      typedef float _m02_type;
      _m02_type m02;
      typedef float _m30_type;
      _m30_type m30;
      typedef float _m21_type;
      _m21_type m21;
      typedef float _m12_type;
      _m12_type m12;
      typedef float _m03_type;
      _m03_type m03;
      typedef float _mu20_type;
      _mu20_type mu20;
      typedef float _mu11_type;
      _mu11_type mu11;
      typedef float _mu02_type;
      _mu02_type mu02;
      typedef float _mu30_type;
      _mu30_type mu30;
      typedef float _mu21_type;
      _mu21_type mu21;
      typedef float _mu12_type;
      _mu12_type mu12;
      typedef float _mu03_type;
      _mu03_type mu03;
      typedef float _nu20_type;
      _nu20_type nu20;
      typedef float _nu11_type;
      _nu11_type nu11;
      typedef float _nu02_type;
      _nu02_type nu02;
      typedef float _nu30_type;
      _nu30_type nu30;
      typedef float _nu21_type;
      _nu21_type nu21;
      typedef float _nu12_type;
      _nu12_type nu12;
      typedef float _nu03_type;
      _nu03_type nu03;
      typedef opencv_apps::Point2D _center_type;
      _center_type center;
      typedef float _length_type;
      _length_type length;
      typedef float _area_type;
      _area_type area;

    Moment():
      m00(0),
      m10(0),
      m01(0),
      m20(0),
      m11(0),
      m02(0),
      m30(0),
      m21(0),
      m12(0),
      m03(0),
      mu20(0),
      mu11(0),
      mu02(0),
      mu30(0),
      mu21(0),
      mu12(0),
      mu03(0),
      nu20(0),
      nu11(0),
      nu02(0),
      nu30(0),
      nu21(0),
      nu12(0),
      nu03(0),
      center(),
      length(0),
      area(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += serializeAvrFloat64(outbuffer + offset, this->m00);
      offset += serializeAvrFloat64(outbuffer + offset, this->m10);
      offset += serializeAvrFloat64(outbuffer + offset, this->m01);
      offset += serializeAvrFloat64(outbuffer + offset, this->m20);
      offset += serializeAvrFloat64(outbuffer + offset, this->m11);
      offset += serializeAvrFloat64(outbuffer + offset, this->m02);
      offset += serializeAvrFloat64(outbuffer + offset, this->m30);
      offset += serializeAvrFloat64(outbuffer + offset, this->m21);
      offset += serializeAvrFloat64(outbuffer + offset, this->m12);
      offset += serializeAvrFloat64(outbuffer + offset, this->m03);
      offset += serializeAvrFloat64(outbuffer + offset, this->mu20);
      offset += serializeAvrFloat64(outbuffer + offset, this->mu11);
      offset += serializeAvrFloat64(outbuffer + offset, this->mu02);
      offset += serializeAvrFloat64(outbuffer + offset, this->mu30);
      offset += serializeAvrFloat64(outbuffer + offset, this->mu21);
      offset += serializeAvrFloat64(outbuffer + offset, this->mu12);
      offset += serializeAvrFloat64(outbuffer + offset, this->mu03);
      offset += serializeAvrFloat64(outbuffer + offset, this->nu20);
      offset += serializeAvrFloat64(outbuffer + offset, this->nu11);
      offset += serializeAvrFloat64(outbuffer + offset, this->nu02);
      offset += serializeAvrFloat64(outbuffer + offset, this->nu30);
      offset += serializeAvrFloat64(outbuffer + offset, this->nu21);
      offset += serializeAvrFloat64(outbuffer + offset, this->nu12);
      offset += serializeAvrFloat64(outbuffer + offset, this->nu03);
      offset += this->center.serialize(outbuffer + offset);
      offset += serializeAvrFloat64(outbuffer + offset, this->length);
      offset += serializeAvrFloat64(outbuffer + offset, this->area);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->m00));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->m10));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->m01));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->m20));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->m11));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->m02));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->m30));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->m21));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->m12));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->m03));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->mu20));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->mu11));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->mu02));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->mu30));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->mu21));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->mu12));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->mu03));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->nu20));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->nu11));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->nu02));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->nu30));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->nu21));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->nu12));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->nu03));
      offset += this->center.deserialize(inbuffer + offset);
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->length));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->area));
     return offset;
    }

    virtual const char * getType() override { return "opencv_apps/Moment"; };
    virtual const char * getMD5() override { return "560ee3fabfffb4ed4155742d6db8a03c"; };

  };

}
#endif
