#ifndef _ROS_mongodb_store_msgs_MoveEntriesResult_h
#define _ROS_mongodb_store_msgs_MoveEntriesResult_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace mongodb_store_msgs
{

  class MoveEntriesResult : public ros::Msg
  {
    public:

    MoveEntriesResult()
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

    virtual const char * getType() override { return "mongodb_store_msgs/MoveEntriesResult"; };
    virtual const char * getMD5() override { return "d41d8cd98f00b204e9800998ecf8427e"; };

  };

}
#endif
