#ifndef _ROS_switchbot_ros_DeviceArray_h
#define _ROS_switchbot_ros_DeviceArray_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "switchbot_ros/Device.h"

namespace switchbot_ros
{

  class DeviceArray : public ros::Msg
  {
    public:
      uint32_t devices_length;
      typedef switchbot_ros::Device _devices_type;
      _devices_type st_devices;
      _devices_type * devices;

    DeviceArray():
      devices_length(0), st_devices(), devices(nullptr)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->devices_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->devices_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->devices_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->devices_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->devices_length);
      for( uint32_t i = 0; i < devices_length; i++){
      offset += this->devices[i].serialize(outbuffer + offset);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      uint32_t devices_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      devices_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      devices_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      devices_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->devices_length);
      if(devices_lengthT > devices_length)
        this->devices = (switchbot_ros::Device*)realloc(this->devices, devices_lengthT * sizeof(switchbot_ros::Device));
      devices_length = devices_lengthT;
      for( uint32_t i = 0; i < devices_length; i++){
      offset += this->st_devices.deserialize(inbuffer + offset);
        memcpy( &(this->devices[i]), &(this->st_devices), sizeof(switchbot_ros::Device));
      }
     return offset;
    }

    virtual const char * getType() override { return "switchbot_ros/DeviceArray"; };
    virtual const char * getMD5() override { return "24b7884c0fb68c7417b906902c32b745"; };

  };

}
#endif
