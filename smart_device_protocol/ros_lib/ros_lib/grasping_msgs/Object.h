#ifndef _ROS_grasping_msgs_Object_h
#define _ROS_grasping_msgs_Object_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "grasping_msgs/ObjectProperty.h"
#include "sensor_msgs/PointCloud2.h"
#include "shape_msgs/SolidPrimitive.h"
#include "geometry_msgs/Pose.h"
#include "shape_msgs/Mesh.h"
#include "shape_msgs/Plane.h"

namespace grasping_msgs
{

  class Object : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef const char* _name_type;
      _name_type name;
      typedef const char* _support_surface_type;
      _support_surface_type support_surface;
      uint32_t properties_length;
      typedef grasping_msgs::ObjectProperty _properties_type;
      _properties_type st_properties;
      _properties_type * properties;
      typedef sensor_msgs::PointCloud2 _point_cluster_type;
      _point_cluster_type point_cluster;
      uint32_t primitives_length;
      typedef shape_msgs::SolidPrimitive _primitives_type;
      _primitives_type st_primitives;
      _primitives_type * primitives;
      uint32_t primitive_poses_length;
      typedef geometry_msgs::Pose _primitive_poses_type;
      _primitive_poses_type st_primitive_poses;
      _primitive_poses_type * primitive_poses;
      uint32_t meshes_length;
      typedef shape_msgs::Mesh _meshes_type;
      _meshes_type st_meshes;
      _meshes_type * meshes;
      uint32_t mesh_poses_length;
      typedef geometry_msgs::Pose _mesh_poses_type;
      _mesh_poses_type st_mesh_poses;
      _mesh_poses_type * mesh_poses;
      typedef shape_msgs::Plane _surface_type;
      _surface_type surface;

    Object():
      header(),
      name(""),
      support_surface(""),
      properties_length(0), st_properties(), properties(nullptr),
      point_cluster(),
      primitives_length(0), st_primitives(), primitives(nullptr),
      primitive_poses_length(0), st_primitive_poses(), primitive_poses(nullptr),
      meshes_length(0), st_meshes(), meshes(nullptr),
      mesh_poses_length(0), st_mesh_poses(), mesh_poses(nullptr),
      surface()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      uint32_t length_name = strlen(this->name);
      varToArr(outbuffer + offset, length_name);
      offset += 4;
      memcpy(outbuffer + offset, this->name, length_name);
      offset += length_name;
      uint32_t length_support_surface = strlen(this->support_surface);
      varToArr(outbuffer + offset, length_support_surface);
      offset += 4;
      memcpy(outbuffer + offset, this->support_surface, length_support_surface);
      offset += length_support_surface;
      *(outbuffer + offset + 0) = (this->properties_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->properties_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->properties_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->properties_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->properties_length);
      for( uint32_t i = 0; i < properties_length; i++){
      offset += this->properties[i].serialize(outbuffer + offset);
      }
      offset += this->point_cluster.serialize(outbuffer + offset);
      *(outbuffer + offset + 0) = (this->primitives_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->primitives_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->primitives_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->primitives_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->primitives_length);
      for( uint32_t i = 0; i < primitives_length; i++){
      offset += this->primitives[i].serialize(outbuffer + offset);
      }
      *(outbuffer + offset + 0) = (this->primitive_poses_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->primitive_poses_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->primitive_poses_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->primitive_poses_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->primitive_poses_length);
      for( uint32_t i = 0; i < primitive_poses_length; i++){
      offset += this->primitive_poses[i].serialize(outbuffer + offset);
      }
      *(outbuffer + offset + 0) = (this->meshes_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->meshes_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->meshes_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->meshes_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->meshes_length);
      for( uint32_t i = 0; i < meshes_length; i++){
      offset += this->meshes[i].serialize(outbuffer + offset);
      }
      *(outbuffer + offset + 0) = (this->mesh_poses_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->mesh_poses_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->mesh_poses_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->mesh_poses_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->mesh_poses_length);
      for( uint32_t i = 0; i < mesh_poses_length; i++){
      offset += this->mesh_poses[i].serialize(outbuffer + offset);
      }
      offset += this->surface.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      uint32_t length_name;
      arrToVar(length_name, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_name; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_name-1]=0;
      this->name = (char *)(inbuffer + offset-1);
      offset += length_name;
      uint32_t length_support_surface;
      arrToVar(length_support_surface, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_support_surface; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_support_surface-1]=0;
      this->support_surface = (char *)(inbuffer + offset-1);
      offset += length_support_surface;
      uint32_t properties_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      properties_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      properties_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      properties_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->properties_length);
      if(properties_lengthT > properties_length)
        this->properties = (grasping_msgs::ObjectProperty*)realloc(this->properties, properties_lengthT * sizeof(grasping_msgs::ObjectProperty));
      properties_length = properties_lengthT;
      for( uint32_t i = 0; i < properties_length; i++){
      offset += this->st_properties.deserialize(inbuffer + offset);
        memcpy( &(this->properties[i]), &(this->st_properties), sizeof(grasping_msgs::ObjectProperty));
      }
      offset += this->point_cluster.deserialize(inbuffer + offset);
      uint32_t primitives_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      primitives_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      primitives_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      primitives_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->primitives_length);
      if(primitives_lengthT > primitives_length)
        this->primitives = (shape_msgs::SolidPrimitive*)realloc(this->primitives, primitives_lengthT * sizeof(shape_msgs::SolidPrimitive));
      primitives_length = primitives_lengthT;
      for( uint32_t i = 0; i < primitives_length; i++){
      offset += this->st_primitives.deserialize(inbuffer + offset);
        memcpy( &(this->primitives[i]), &(this->st_primitives), sizeof(shape_msgs::SolidPrimitive));
      }
      uint32_t primitive_poses_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      primitive_poses_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      primitive_poses_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      primitive_poses_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->primitive_poses_length);
      if(primitive_poses_lengthT > primitive_poses_length)
        this->primitive_poses = (geometry_msgs::Pose*)realloc(this->primitive_poses, primitive_poses_lengthT * sizeof(geometry_msgs::Pose));
      primitive_poses_length = primitive_poses_lengthT;
      for( uint32_t i = 0; i < primitive_poses_length; i++){
      offset += this->st_primitive_poses.deserialize(inbuffer + offset);
        memcpy( &(this->primitive_poses[i]), &(this->st_primitive_poses), sizeof(geometry_msgs::Pose));
      }
      uint32_t meshes_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      meshes_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      meshes_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      meshes_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->meshes_length);
      if(meshes_lengthT > meshes_length)
        this->meshes = (shape_msgs::Mesh*)realloc(this->meshes, meshes_lengthT * sizeof(shape_msgs::Mesh));
      meshes_length = meshes_lengthT;
      for( uint32_t i = 0; i < meshes_length; i++){
      offset += this->st_meshes.deserialize(inbuffer + offset);
        memcpy( &(this->meshes[i]), &(this->st_meshes), sizeof(shape_msgs::Mesh));
      }
      uint32_t mesh_poses_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      mesh_poses_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      mesh_poses_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      mesh_poses_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->mesh_poses_length);
      if(mesh_poses_lengthT > mesh_poses_length)
        this->mesh_poses = (geometry_msgs::Pose*)realloc(this->mesh_poses, mesh_poses_lengthT * sizeof(geometry_msgs::Pose));
      mesh_poses_length = mesh_poses_lengthT;
      for( uint32_t i = 0; i < mesh_poses_length; i++){
      offset += this->st_mesh_poses.deserialize(inbuffer + offset);
        memcpy( &(this->mesh_poses[i]), &(this->st_mesh_poses), sizeof(geometry_msgs::Pose));
      }
      offset += this->surface.deserialize(inbuffer + offset);
     return offset;
    }

    virtual const char * getType() override { return "grasping_msgs/Object"; };
    virtual const char * getMD5() override { return "0770e300363d18954f6fd25963e4d536"; };

  };

}
#endif
