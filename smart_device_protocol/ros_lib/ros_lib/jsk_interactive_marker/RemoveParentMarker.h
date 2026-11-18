#ifndef _ROS_SERVICE_RemoveParentMarker_h
#define _ROS_SERVICE_RemoveParentMarker_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace jsk_interactive_marker
{

static const char REMOVEPARENTMARKER[] = "jsk_interactive_marker/RemoveParentMarker";

  class RemoveParentMarkerRequest : public ros::Msg
  {
    public:
      typedef const char* _child_marker_name_type;
      _child_marker_name_type child_marker_name;

    RemoveParentMarkerRequest():
      child_marker_name("")
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      uint32_t length_child_marker_name = strlen(this->child_marker_name);
      varToArr(outbuffer + offset, length_child_marker_name);
      offset += 4;
      memcpy(outbuffer + offset, this->child_marker_name, length_child_marker_name);
      offset += length_child_marker_name;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      uint32_t length_child_marker_name;
      arrToVar(length_child_marker_name, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_child_marker_name; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_child_marker_name-1]=0;
      this->child_marker_name = (char *)(inbuffer + offset-1);
      offset += length_child_marker_name;
     return offset;
    }

    virtual const char * getType() override { return REMOVEPARENTMARKER; };
    virtual const char * getMD5() override { return "97181b161a4bd485fbd404baffdcfbf4"; };

  };

  class RemoveParentMarkerResponse : public ros::Msg
  {
    public:

    RemoveParentMarkerResponse()
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

    virtual const char * getType() override { return REMOVEPARENTMARKER; };
    virtual const char * getMD5() override { return "d41d8cd98f00b204e9800998ecf8427e"; };

  };

  class RemoveParentMarker {
    public:
    typedef RemoveParentMarkerRequest Request;
    typedef RemoveParentMarkerResponse Response;
  };

}
#endif
