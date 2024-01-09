#ifndef _ROS_SERVICE_SetTransformableMarkerPose_h
#define _ROS_SERVICE_SetTransformableMarkerPose_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "geometry_msgs/PoseStamped.h"

namespace jsk_interactive_marker
{

static const char SETTRANSFORMABLEMARKERPOSE[] = "jsk_interactive_marker/SetTransformableMarkerPose";

  class SetTransformableMarkerPoseRequest : public ros::Msg
  {
    public:
      typedef const char* _target_name_type;
      _target_name_type target_name;
      typedef geometry_msgs::PoseStamped _pose_stamped_type;
      _pose_stamped_type pose_stamped;

    SetTransformableMarkerPoseRequest():
      target_name(""),
      pose_stamped()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      uint32_t length_target_name = strlen(this->target_name);
      varToArr(outbuffer + offset, length_target_name);
      offset += 4;
      memcpy(outbuffer + offset, this->target_name, length_target_name);
      offset += length_target_name;
      offset += this->pose_stamped.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      uint32_t length_target_name;
      arrToVar(length_target_name, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_target_name; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_target_name-1]=0;
      this->target_name = (char *)(inbuffer + offset-1);
      offset += length_target_name;
      offset += this->pose_stamped.deserialize(inbuffer + offset);
     return offset;
    }

    virtual const char * getType() override { return SETTRANSFORMABLEMARKERPOSE; };
    virtual const char * getMD5() override { return "e19607b29b4528e87feff290fe261191"; };

  };

  class SetTransformableMarkerPoseResponse : public ros::Msg
  {
    public:

    SetTransformableMarkerPoseResponse()
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

    virtual const char * getType() override { return SETTRANSFORMABLEMARKERPOSE; };
    virtual const char * getMD5() override { return "d41d8cd98f00b204e9800998ecf8427e"; };

  };

  class SetTransformableMarkerPose {
    public:
    typedef SetTransformableMarkerPoseRequest Request;
    typedef SetTransformableMarkerPoseResponse Response;
  };

}
#endif
