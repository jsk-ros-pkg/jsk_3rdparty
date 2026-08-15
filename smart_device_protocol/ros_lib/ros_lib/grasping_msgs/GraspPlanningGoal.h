#ifndef _ROS_grasping_msgs_GraspPlanningGoal_h
#define _ROS_grasping_msgs_GraspPlanningGoal_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "grasping_msgs/Object.h"

namespace grasping_msgs
{

  class GraspPlanningGoal : public ros::Msg
  {
    public:
      typedef grasping_msgs::Object _object_type;
      _object_type object;
      typedef const char* _group_name_type;
      _group_name_type group_name;

    GraspPlanningGoal():
      object(),
      group_name("")
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->object.serialize(outbuffer + offset);
      uint32_t length_group_name = strlen(this->group_name);
      varToArr(outbuffer + offset, length_group_name);
      offset += 4;
      memcpy(outbuffer + offset, this->group_name, length_group_name);
      offset += length_group_name;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += this->object.deserialize(inbuffer + offset);
      uint32_t length_group_name;
      arrToVar(length_group_name, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_group_name; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_group_name-1]=0;
      this->group_name = (char *)(inbuffer + offset-1);
      offset += length_group_name;
     return offset;
    }

    virtual const char * getType() override { return "grasping_msgs/GraspPlanningGoal"; };
    virtual const char * getMD5() override { return "1c3f3ed2a31c4c865c3032a4789c0df9"; };

  };

}
#endif
