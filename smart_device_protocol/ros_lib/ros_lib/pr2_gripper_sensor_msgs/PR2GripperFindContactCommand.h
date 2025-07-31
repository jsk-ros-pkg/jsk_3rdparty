#ifndef _ROS_pr2_gripper_sensor_msgs_PR2GripperFindContactCommand_h
#define _ROS_pr2_gripper_sensor_msgs_PR2GripperFindContactCommand_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace pr2_gripper_sensor_msgs
{

  class PR2GripperFindContactCommand : public ros::Msg
  {
    public:
      typedef bool _zero_fingertip_sensors_type;
      _zero_fingertip_sensors_type zero_fingertip_sensors;
      typedef int8_t _contact_conditions_type;
      _contact_conditions_type contact_conditions;
      enum { BOTH =  0    };
      enum { LEFT =  1    };
      enum { RIGHT =  2   };
      enum { EITHER =  3  };

    PR2GripperFindContactCommand():
      zero_fingertip_sensors(0),
      contact_conditions(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_zero_fingertip_sensors;
      u_zero_fingertip_sensors.real = this->zero_fingertip_sensors;
      *(outbuffer + offset + 0) = (u_zero_fingertip_sensors.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->zero_fingertip_sensors);
      union {
        int8_t real;
        uint8_t base;
      } u_contact_conditions;
      u_contact_conditions.real = this->contact_conditions;
      *(outbuffer + offset + 0) = (u_contact_conditions.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->contact_conditions);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_zero_fingertip_sensors;
      u_zero_fingertip_sensors.base = 0;
      u_zero_fingertip_sensors.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->zero_fingertip_sensors = u_zero_fingertip_sensors.real;
      offset += sizeof(this->zero_fingertip_sensors);
      union {
        int8_t real;
        uint8_t base;
      } u_contact_conditions;
      u_contact_conditions.base = 0;
      u_contact_conditions.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->contact_conditions = u_contact_conditions.real;
      offset += sizeof(this->contact_conditions);
     return offset;
    }

    virtual const char * getType() override { return "pr2_gripper_sensor_msgs/PR2GripperFindContactCommand"; };
    virtual const char * getMD5() override { return "4a38a1a8e495aae86921ef2b292ec260"; };

  };

}
#endif
