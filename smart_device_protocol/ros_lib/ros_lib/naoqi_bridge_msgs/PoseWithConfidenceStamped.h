#ifndef _ROS_naoqi_bridge_msgs_PoseWithConfidenceStamped_h
#define _ROS_naoqi_bridge_msgs_PoseWithConfidenceStamped_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "geometry_msgs/Pose.h"

namespace naoqi_bridge_msgs
{

  class PoseWithConfidenceStamped : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef geometry_msgs::Pose _pose_type;
      _pose_type pose;
      typedef float _confidence_index_type;
      _confidence_index_type confidence_index;

    PoseWithConfidenceStamped():
      header(),
      pose(),
      confidence_index(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      offset += this->pose.serialize(outbuffer + offset);
      union {
        float real;
        uint32_t base;
      } u_confidence_index;
      u_confidence_index.real = this->confidence_index;
      *(outbuffer + offset + 0) = (u_confidence_index.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_confidence_index.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_confidence_index.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_confidence_index.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->confidence_index);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      offset += this->pose.deserialize(inbuffer + offset);
      union {
        float real;
        uint32_t base;
      } u_confidence_index;
      u_confidence_index.base = 0;
      u_confidence_index.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_confidence_index.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_confidence_index.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_confidence_index.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->confidence_index = u_confidence_index.real;
      offset += sizeof(this->confidence_index);
     return offset;
    }

    virtual const char * getType() override { return "naoqi_bridge_msgs/PoseWithConfidenceStamped"; };
    virtual const char * getMD5() override { return "7503bfd0de35644d373258ab08dd1e84"; };

  };

}
#endif
