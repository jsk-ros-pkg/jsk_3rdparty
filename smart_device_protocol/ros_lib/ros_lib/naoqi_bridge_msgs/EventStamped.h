#ifndef _ROS_naoqi_bridge_msgs_EventStamped_h
#define _ROS_naoqi_bridge_msgs_EventStamped_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "std_msgs/String.h"

namespace naoqi_bridge_msgs
{

  class EventStamped : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef std_msgs::String _name_type;
      _name_type name;
      typedef std_msgs::String _data_type;
      _data_type data;

    EventStamped():
      header(),
      name(),
      data()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      offset += this->name.serialize(outbuffer + offset);
      offset += this->data.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      offset += this->name.deserialize(inbuffer + offset);
      offset += this->data.deserialize(inbuffer + offset);
     return offset;
    }

    virtual const char * getType() override { return "naoqi_bridge_msgs/EventStamped"; };
    virtual const char * getMD5() override { return "da9da7dab2e8376f0a588b6d053ac972"; };

  };

}
#endif
