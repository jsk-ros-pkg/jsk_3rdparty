#ifndef _ROS_grasping_msgs_FindGraspableObjectsResult_h
#define _ROS_grasping_msgs_FindGraspableObjectsResult_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "grasping_msgs/GraspableObject.h"
#include "grasping_msgs/Object.h"

namespace grasping_msgs
{

  class FindGraspableObjectsResult : public ros::Msg
  {
    public:
      uint32_t objects_length;
      typedef grasping_msgs::GraspableObject _objects_type;
      _objects_type st_objects;
      _objects_type * objects;
      uint32_t support_surfaces_length;
      typedef grasping_msgs::Object _support_surfaces_type;
      _support_surfaces_type st_support_surfaces;
      _support_surfaces_type * support_surfaces;

    FindGraspableObjectsResult():
      objects_length(0), st_objects(), objects(nullptr),
      support_surfaces_length(0), st_support_surfaces(), support_surfaces(nullptr)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->objects_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->objects_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->objects_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->objects_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->objects_length);
      for( uint32_t i = 0; i < objects_length; i++){
      offset += this->objects[i].serialize(outbuffer + offset);
      }
      *(outbuffer + offset + 0) = (this->support_surfaces_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->support_surfaces_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->support_surfaces_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->support_surfaces_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->support_surfaces_length);
      for( uint32_t i = 0; i < support_surfaces_length; i++){
      offset += this->support_surfaces[i].serialize(outbuffer + offset);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      uint32_t objects_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      objects_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      objects_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      objects_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->objects_length);
      if(objects_lengthT > objects_length)
        this->objects = (grasping_msgs::GraspableObject*)realloc(this->objects, objects_lengthT * sizeof(grasping_msgs::GraspableObject));
      objects_length = objects_lengthT;
      for( uint32_t i = 0; i < objects_length; i++){
      offset += this->st_objects.deserialize(inbuffer + offset);
        memcpy( &(this->objects[i]), &(this->st_objects), sizeof(grasping_msgs::GraspableObject));
      }
      uint32_t support_surfaces_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      support_surfaces_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      support_surfaces_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      support_surfaces_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->support_surfaces_length);
      if(support_surfaces_lengthT > support_surfaces_length)
        this->support_surfaces = (grasping_msgs::Object*)realloc(this->support_surfaces, support_surfaces_lengthT * sizeof(grasping_msgs::Object));
      support_surfaces_length = support_surfaces_lengthT;
      for( uint32_t i = 0; i < support_surfaces_length; i++){
      offset += this->st_support_surfaces.deserialize(inbuffer + offset);
        memcpy( &(this->support_surfaces[i]), &(this->st_support_surfaces), sizeof(grasping_msgs::Object));
      }
     return offset;
    }

    virtual const char * getType() override { return "grasping_msgs/FindGraspableObjectsResult"; };
    virtual const char * getMD5() override { return "b0e2a5b10c524db813b26378dd6d8559"; };

  };

}
#endif
