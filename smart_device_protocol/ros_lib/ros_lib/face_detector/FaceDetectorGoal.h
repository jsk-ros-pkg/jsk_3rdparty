#ifndef _ROS_face_detector_FaceDetectorGoal_h
#define _ROS_face_detector_FaceDetectorGoal_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace face_detector
{

  class FaceDetectorGoal : public ros::Msg
  {
    public:

    FaceDetectorGoal()
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

    virtual const char * getType() override { return "face_detector/FaceDetectorGoal"; };
    virtual const char * getMD5() override { return "d41d8cd98f00b204e9800998ecf8427e"; };

  };

}
#endif
