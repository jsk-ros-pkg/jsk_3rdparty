#ifndef _ROS_pr2_gripper_sensor_msgs_PR2GripperPressureData_h
#define _ROS_pr2_gripper_sensor_msgs_PR2GripperPressureData_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace pr2_gripper_sensor_msgs
{

  class PR2GripperPressureData : public ros::Msg
  {
    public:
      float pressure_left[22];
      float pressure_right[22];
      typedef float _rostime_type;
      _rostime_type rostime;

    PR2GripperPressureData():
      pressure_left(),
      pressure_right(),
      rostime(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      for( uint32_t i = 0; i < 22; i++){
      offset += serializeAvrFloat64(outbuffer + offset, this->pressure_left[i]);
      }
      for( uint32_t i = 0; i < 22; i++){
      offset += serializeAvrFloat64(outbuffer + offset, this->pressure_right[i]);
      }
      offset += serializeAvrFloat64(outbuffer + offset, this->rostime);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      for( uint32_t i = 0; i < 22; i++){
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->pressure_left[i]));
      }
      for( uint32_t i = 0; i < 22; i++){
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->pressure_right[i]));
      }
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->rostime));
     return offset;
    }

    virtual const char * getType() override { return "pr2_gripper_sensor_msgs/PR2GripperPressureData"; };
    virtual const char * getMD5() override { return "b69255f5117bf05fdcd1e83d4e6ab779"; };

  };

}
#endif
