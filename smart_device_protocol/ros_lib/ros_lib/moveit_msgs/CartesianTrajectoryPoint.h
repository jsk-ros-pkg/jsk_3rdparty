#ifndef _ROS_moveit_msgs_CartesianTrajectoryPoint_h
#define _ROS_moveit_msgs_CartesianTrajectoryPoint_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "moveit_msgs/CartesianPoint.h"
#include "ros/duration.h"

namespace moveit_msgs
{

  class CartesianTrajectoryPoint : public ros::Msg
  {
    public:
      typedef moveit_msgs::CartesianPoint _point_type;
      _point_type point;
      typedef ros::Duration _time_from_start_type;
      _time_from_start_type time_from_start;

    CartesianTrajectoryPoint():
      point(),
      time_from_start()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->point.serialize(outbuffer + offset);
      *(outbuffer + offset + 0) = (this->time_from_start.sec >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->time_from_start.sec >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->time_from_start.sec >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->time_from_start.sec >> (8 * 3)) & 0xFF;
      offset += sizeof(this->time_from_start.sec);
      *(outbuffer + offset + 0) = (this->time_from_start.nsec >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->time_from_start.nsec >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->time_from_start.nsec >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->time_from_start.nsec >> (8 * 3)) & 0xFF;
      offset += sizeof(this->time_from_start.nsec);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += this->point.deserialize(inbuffer + offset);
      this->time_from_start.sec =  ((uint32_t) (*(inbuffer + offset)));
      this->time_from_start.sec |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->time_from_start.sec |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->time_from_start.sec |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->time_from_start.sec);
      this->time_from_start.nsec =  ((uint32_t) (*(inbuffer + offset)));
      this->time_from_start.nsec |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->time_from_start.nsec |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->time_from_start.nsec |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->time_from_start.nsec);
     return offset;
    }

    virtual const char * getType() override { return "moveit_msgs/CartesianTrajectoryPoint"; };
    virtual const char * getMD5() override { return "e996ea294f646e6911b3e85e624f5acf"; };

  };

}
#endif
