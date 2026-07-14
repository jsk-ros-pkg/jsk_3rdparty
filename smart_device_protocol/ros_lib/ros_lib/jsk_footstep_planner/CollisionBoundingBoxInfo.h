#ifndef _ROS_SERVICE_CollisionBoundingBoxInfo_h
#define _ROS_SERVICE_CollisionBoundingBoxInfo_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Vector3.h"

namespace jsk_footstep_planner
{

static const char COLLISIONBOUNDINGBOXINFO[] = "jsk_footstep_planner/CollisionBoundingBoxInfo";

  class CollisionBoundingBoxInfoRequest : public ros::Msg
  {
    public:

    CollisionBoundingBoxInfoRequest()
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

    virtual const char * getType() override { return COLLISIONBOUNDINGBOXINFO; };
    virtual const char * getMD5() override { return "d41d8cd98f00b204e9800998ecf8427e"; };

  };

  class CollisionBoundingBoxInfoResponse : public ros::Msg
  {
    public:
      typedef geometry_msgs::Vector3 _box_dimensions_type;
      _box_dimensions_type box_dimensions;
      typedef geometry_msgs::Pose _box_offset_type;
      _box_offset_type box_offset;

    CollisionBoundingBoxInfoResponse():
      box_dimensions(),
      box_offset()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->box_dimensions.serialize(outbuffer + offset);
      offset += this->box_offset.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += this->box_dimensions.deserialize(inbuffer + offset);
      offset += this->box_offset.deserialize(inbuffer + offset);
     return offset;
    }

    virtual const char * getType() override { return COLLISIONBOUNDINGBOXINFO; };
    virtual const char * getMD5() override { return "2ea386819a2c4e5466f51a04205f588b"; };

  };

  class CollisionBoundingBoxInfo {
    public:
    typedef CollisionBoundingBoxInfoRequest Request;
    typedef CollisionBoundingBoxInfoResponse Response;
  };

}
#endif
