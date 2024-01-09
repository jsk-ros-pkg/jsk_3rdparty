#ifndef _ROS_SERVICE_GetTransformableMarkerPose_h
#define _ROS_SERVICE_GetTransformableMarkerPose_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "geometry_msgs/PoseStamped.h"

namespace jsk_interactive_marker
{

static const char GETTRANSFORMABLEMARKERPOSE[] = "jsk_interactive_marker/GetTransformableMarkerPose";

  class GetTransformableMarkerPoseRequest : public ros::Msg
  {
    public:
      typedef const char* _target_name_type;
      _target_name_type target_name;

    GetTransformableMarkerPoseRequest():
      target_name("")
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
     return offset;
    }

    virtual const char * getType() override { return GETTRANSFORMABLEMARKERPOSE; };
    virtual const char * getMD5() override { return "2008933b3c7227647cbe00c74682331a"; };

  };

  class GetTransformableMarkerPoseResponse : public ros::Msg
  {
    public:
      typedef geometry_msgs::PoseStamped _pose_stamped_type;
      _pose_stamped_type pose_stamped;

    GetTransformableMarkerPoseResponse():
      pose_stamped()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->pose_stamped.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += this->pose_stamped.deserialize(inbuffer + offset);
     return offset;
    }

    virtual const char * getType() override { return GETTRANSFORMABLEMARKERPOSE; };
    virtual const char * getMD5() override { return "a6cf8bca3220fd47abb2c1783444110d"; };

  };

  class GetTransformableMarkerPose {
    public:
    typedef GetTransformableMarkerPoseRequest Request;
    typedef GetTransformableMarkerPoseResponse Response;
  };

}
#endif
