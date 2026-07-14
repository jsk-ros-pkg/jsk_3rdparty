#ifndef _ROS_SERVICE_ApplyPlanningScene_h
#define _ROS_SERVICE_ApplyPlanningScene_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "moveit_msgs/PlanningScene.h"

namespace moveit_msgs
{

static const char APPLYPLANNINGSCENE[] = "moveit_msgs/ApplyPlanningScene";

  class ApplyPlanningSceneRequest : public ros::Msg
  {
    public:
      typedef moveit_msgs::PlanningScene _scene_type;
      _scene_type scene;

    ApplyPlanningSceneRequest():
      scene()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->scene.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += this->scene.deserialize(inbuffer + offset);
     return offset;
    }

    virtual const char * getType() override { return APPLYPLANNINGSCENE; };
    virtual const char * getMD5() override { return "532b54e7c502b73178625025da63b084"; };

  };

  class ApplyPlanningSceneResponse : public ros::Msg
  {
    public:
      typedef bool _success_type;
      _success_type success;

    ApplyPlanningSceneResponse():
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

    virtual const char * getType() override { return APPLYPLANNINGSCENE; };
    virtual const char * getMD5() override { return "358e233cde0c8a8bcfea4ce193f8fc15"; };

  };

  class ApplyPlanningScene {
    public:
    typedef ApplyPlanningSceneRequest Request;
    typedef ApplyPlanningSceneResponse Response;
  };

}
#endif
