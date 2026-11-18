#ifndef _ROS_SERVICE_IndexRequest_h
#define _ROS_SERVICE_IndexRequest_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "jsk_recognition_msgs/Int32Stamped.h"

namespace jsk_interactive_marker
{

static const char INDEXREQUEST[] = "jsk_interactive_marker/IndexRequest";

  class IndexRequestRequest : public ros::Msg
  {
    public:
      typedef jsk_recognition_msgs::Int32Stamped _index_type;
      _index_type index;

    IndexRequestRequest():
      index()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->index.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += this->index.deserialize(inbuffer + offset);
     return offset;
    }

    virtual const char * getType() override { return INDEXREQUEST; };
    virtual const char * getMD5() override { return "e7767d85a4611e638acb5e4f67adbc5a"; };

  };

  class IndexRequestResponse : public ros::Msg
  {
    public:

    IndexRequestResponse()
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

    virtual const char * getType() override { return INDEXREQUEST; };
    virtual const char * getMD5() override { return "d41d8cd98f00b204e9800998ecf8427e"; };

  };

  class IndexRequest {
    public:
    typedef IndexRequestRequest Request;
    typedef IndexRequestResponse Response;
  };

}
#endif
