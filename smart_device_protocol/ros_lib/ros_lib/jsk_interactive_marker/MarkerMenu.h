#ifndef _ROS_jsk_interactive_marker_MarkerMenu_h
#define _ROS_jsk_interactive_marker_MarkerMenu_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace jsk_interactive_marker
{

  class MarkerMenu : public ros::Msg
  {
    public:
      typedef int8_t _menu_type;
      _menu_type menu;
      typedef int8_t _type_type;
      _type_type type;
      typedef const char* _marker_name_type;
      _marker_name_type marker_name;
      enum { MOVE = 0 };
      enum { FORCE_MOVE = 1 };
      enum { SET_ORIGIN = 2 };
      enum { SET_ORIGIN_RHAND = 3 };
      enum { SET_ORIGIN_LHAND = 4 };
      enum { RESET_COORDS = 5 };
      enum { DELETE_FORCE = 6 };
      enum { PUBLISH_MARKER = 7 };
      enum { JOINT_MOVE = 8 };
      enum { RESET_JOINT = 9 };
      enum { SET_MOVE_RARM = 10 };
      enum { SET_MOVE_LARM = 11 };
      enum { SET_MOVE_ARMS = 12 };
      enum { MOVE_CONSTRAINT_T = 13 };
      enum { MOVE_CONSTRAINT_NIL = 14 };
      enum { IK_ROTATION_AXIS_T = 15 };
      enum { IK_ROTATION_AXIS_NIL = 16 };
      enum { USE_TORSO_T = 17 };
      enum { USE_TORSO_NIL = 18 };
      enum { USE_FULLBODY = 19 };
      enum { START_GRASP = 20 };
      enum { HARF_GRASP = 21 };
      enum { STOP_GRASP = 22 };
      enum { HEAD_TARGET_POINT = 30 };
      enum { LOOK_AUTO = 31 };
      enum { MANIP_MODE = 40 };
      enum { PICK = 41 };
      enum { TOUCHIT_EXEC = 42 };
      enum { TOUCHIT_PREV = 43 };
      enum { TOUCHIT_CANCEL = 44 };
      enum { LOOK_RARM = 45 };
      enum { LOOK_LARM = 46 };
      enum { PLAN = 50 };
      enum { EXECUTE = 51 };
      enum { PLAN_EXECUTE = 52 };
      enum { CANCEL_PLAN = 53 };
      enum { GENERAL = 0 };
      enum { HEAD_MARKER = 1 };
      enum { RHAND_MARKER = 2 };
      enum { LHAND_MARKER = 3 };
      enum { RLEG_MARKER = 4 };
      enum { LLEG_MARKER = 5 };
      enum { BASE_MARKER = 6 };
      enum { RFINGER_MARKER = 7 };
      enum { LFINGER_MARKER = 8 };

    MarkerMenu():
      menu(0),
      type(0),
      marker_name("")
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      union {
        int8_t real;
        uint8_t base;
      } u_menu;
      u_menu.real = this->menu;
      *(outbuffer + offset + 0) = (u_menu.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->menu);
      union {
        int8_t real;
        uint8_t base;
      } u_type;
      u_type.real = this->type;
      *(outbuffer + offset + 0) = (u_type.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->type);
      uint32_t length_marker_name = strlen(this->marker_name);
      varToArr(outbuffer + offset, length_marker_name);
      offset += 4;
      memcpy(outbuffer + offset, this->marker_name, length_marker_name);
      offset += length_marker_name;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      union {
        int8_t real;
        uint8_t base;
      } u_menu;
      u_menu.base = 0;
      u_menu.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->menu = u_menu.real;
      offset += sizeof(this->menu);
      union {
        int8_t real;
        uint8_t base;
      } u_type;
      u_type.base = 0;
      u_type.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->type = u_type.real;
      offset += sizeof(this->type);
      uint32_t length_marker_name;
      arrToVar(length_marker_name, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_marker_name; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_marker_name-1]=0;
      this->marker_name = (char *)(inbuffer + offset-1);
      offset += length_marker_name;
     return offset;
    }

    virtual const char * getType() override { return "jsk_interactive_marker/MarkerMenu"; };
    virtual const char * getMD5() override { return "192d3b78eda584051c0d487463f7de74"; };

  };

}
#endif
