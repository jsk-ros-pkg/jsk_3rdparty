#ifndef _ROS_SERVICE_UpdatePointcloudOctomap_h
#define _ROS_SERVICE_UpdatePointcloudOctomap_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "sensor_msgs/PointCloud2.h"

namespace moveit_msgs
{

static const char UPDATEPOINTCLOUDOCTOMAP[] = "moveit_msgs/UpdatePointcloudOctomap";

  class UpdatePointcloudOctomapRequest : public ros::Msg
  {
    public:
      typedef sensor_msgs::PointCloud2 _cloud_type;
      _cloud_type cloud;

    UpdatePointcloudOctomapRequest():
      cloud()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->cloud.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += this->cloud.deserialize(inbuffer + offset);
     return offset;
    }

    virtual const char * getType() override { return UPDATEPOINTCLOUDOCTOMAP; };
    virtual const char * getMD5() override { return "96cec5374164b3b3d1d7ef5d7628a7ed"; };

  };

  class UpdatePointcloudOctomapResponse : public ros::Msg
  {
    public:
      typedef bool _success_type;
      _success_type success;

    UpdatePointcloudOctomapResponse():
      success(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_success;
      u_success.real = this->success;
      *(outbuffer + offset + 0) = (u_success.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->success);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_success;
      u_success.base = 0;
      u_success.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->success = u_success.real;
      offset += sizeof(this->success);
     return offset;
    }

    virtual const char * getType() override { return UPDATEPOINTCLOUDOCTOMAP; };
    virtual const char * getMD5() override { return "358e233cde0c8a8bcfea4ce193f8fc15"; };

  };

  class UpdatePointcloudOctomap {
    public:
    typedef UpdatePointcloudOctomapRequest Request;
    typedef UpdatePointcloudOctomapResponse Response;
  };

}
#endif
