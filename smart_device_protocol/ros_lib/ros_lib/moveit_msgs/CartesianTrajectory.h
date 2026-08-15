#ifndef _ROS_moveit_msgs_CartesianTrajectory_h
#define _ROS_moveit_msgs_CartesianTrajectory_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "moveit_msgs/CartesianTrajectoryPoint.h"

namespace moveit_msgs
{

  class CartesianTrajectory : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef const char* _tracked_frame_type;
      _tracked_frame_type tracked_frame;
      uint32_t points_length;
      typedef moveit_msgs::CartesianTrajectoryPoint _points_type;
      _points_type st_points;
      _points_type * points;

    CartesianTrajectory():
      header(),
      tracked_frame(""),
      points_length(0), st_points(), points(nullptr)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      uint32_t length_tracked_frame = strlen(this->tracked_frame);
      varToArr(outbuffer + offset, length_tracked_frame);
      offset += 4;
      memcpy(outbuffer + offset, this->tracked_frame, length_tracked_frame);
      offset += length_tracked_frame;
      *(outbuffer + offset + 0) = (this->points_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->points_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->points_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->points_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->points_length);
      for( uint32_t i = 0; i < points_length; i++){
      offset += this->points[i].serialize(outbuffer + offset);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      uint32_t length_tracked_frame;
      arrToVar(length_tracked_frame, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_tracked_frame; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_tracked_frame-1]=0;
      this->tracked_frame = (char *)(inbuffer + offset-1);
      offset += length_tracked_frame;
      uint32_t points_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      points_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      points_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      points_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->points_length);
      if(points_lengthT > points_length)
        this->points = (moveit_msgs::CartesianTrajectoryPoint*)realloc(this->points, points_lengthT * sizeof(moveit_msgs::CartesianTrajectoryPoint));
      points_length = points_lengthT;
      for( uint32_t i = 0; i < points_length; i++){
      offset += this->st_points.deserialize(inbuffer + offset);
        memcpy( &(this->points[i]), &(this->st_points), sizeof(moveit_msgs::CartesianTrajectoryPoint));
      }
     return offset;
    }

    virtual const char * getType() override { return "moveit_msgs/CartesianTrajectory"; };
    virtual const char * getMD5() override { return "4886769850ce858fcceba546fd5c793b"; };

  };

}
#endif
