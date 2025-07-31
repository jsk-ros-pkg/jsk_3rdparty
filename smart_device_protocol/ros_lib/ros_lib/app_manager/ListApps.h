#ifndef _ROS_SERVICE_ListApps_h
#define _ROS_SERVICE_ListApps_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "app_manager/App.h"

namespace app_manager
{

static const char LISTAPPS[] = "app_manager/ListApps";

  class ListAppsRequest : public ros::Msg
  {
    public:

    ListAppsRequest()
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

    virtual const char * getType() override { return LISTAPPS; };
    virtual const char * getMD5() override { return "d41d8cd98f00b204e9800998ecf8427e"; };

  };

  class ListAppsResponse : public ros::Msg
  {
    public:
      uint32_t running_apps_length;
      typedef app_manager::App _running_apps_type;
      _running_apps_type st_running_apps;
      _running_apps_type * running_apps;
      uint32_t available_apps_length;
      typedef app_manager::App _available_apps_type;
      _available_apps_type st_available_apps;
      _available_apps_type * available_apps;

    ListAppsResponse():
      running_apps_length(0), st_running_apps(), running_apps(nullptr),
      available_apps_length(0), st_available_apps(), available_apps(nullptr)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->running_apps_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->running_apps_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->running_apps_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->running_apps_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->running_apps_length);
      for( uint32_t i = 0; i < running_apps_length; i++){
      offset += this->running_apps[i].serialize(outbuffer + offset);
      }
      *(outbuffer + offset + 0) = (this->available_apps_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->available_apps_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->available_apps_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->available_apps_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->available_apps_length);
      for( uint32_t i = 0; i < available_apps_length; i++){
      offset += this->available_apps[i].serialize(outbuffer + offset);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      uint32_t running_apps_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      running_apps_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      running_apps_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      running_apps_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->running_apps_length);
      if(running_apps_lengthT > running_apps_length)
        this->running_apps = (app_manager::App*)realloc(this->running_apps, running_apps_lengthT * sizeof(app_manager::App));
      running_apps_length = running_apps_lengthT;
      for( uint32_t i = 0; i < running_apps_length; i++){
      offset += this->st_running_apps.deserialize(inbuffer + offset);
        memcpy( &(this->running_apps[i]), &(this->st_running_apps), sizeof(app_manager::App));
      }
      uint32_t available_apps_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      available_apps_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      available_apps_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      available_apps_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->available_apps_length);
      if(available_apps_lengthT > available_apps_length)
        this->available_apps = (app_manager::App*)realloc(this->available_apps, available_apps_lengthT * sizeof(app_manager::App));
      available_apps_length = available_apps_lengthT;
      for( uint32_t i = 0; i < available_apps_length; i++){
      offset += this->st_available_apps.deserialize(inbuffer + offset);
        memcpy( &(this->available_apps[i]), &(this->st_available_apps), sizeof(app_manager::App));
      }
     return offset;
    }

    virtual const char * getType() override { return LISTAPPS; };
    virtual const char * getMD5() override { return "8a71ede6bf51909653c7c551f462cb30"; };

  };

  class ListApps {
    public:
    typedef ListAppsRequest Request;
    typedef ListAppsResponse Response;
  };

}
#endif
