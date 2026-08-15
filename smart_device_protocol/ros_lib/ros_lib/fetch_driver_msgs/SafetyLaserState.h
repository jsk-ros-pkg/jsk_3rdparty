#ifndef _ROS_fetch_driver_msgs_SafetyLaserState_h
#define _ROS_fetch_driver_msgs_SafetyLaserState_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace fetch_driver_msgs
{

  class SafetyLaserState : public ros::Msg
  {
    public:
      typedef uint8_t _monitoring_case_type;
      _monitoring_case_type monitoring_case;
      typedef bool _protective_field_free_type;
      _protective_field_free_type protective_field_free;
      typedef bool _warning_field1_free_type;
      _warning_field1_free_type warning_field1_free;
      typedef bool _warning_field2_free_type;
      _warning_field2_free_type warning_field2_free;
      typedef bool _contaminated_type;
      _contaminated_type contaminated;

    SafetyLaserState():
      monitoring_case(0),
      protective_field_free(0),
      warning_field1_free(0),
      warning_field2_free(0),
      contaminated(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->monitoring_case >> (8 * 0)) & 0xFF;
      offset += sizeof(this->monitoring_case);
      union {
        bool real;
        uint8_t base;
      } u_protective_field_free;
      u_protective_field_free.real = this->protective_field_free;
      *(outbuffer + offset + 0) = (u_protective_field_free.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->protective_field_free);
      union {
        bool real;
        uint8_t base;
      } u_warning_field1_free;
      u_warning_field1_free.real = this->warning_field1_free;
      *(outbuffer + offset + 0) = (u_warning_field1_free.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->warning_field1_free);
      union {
        bool real;
        uint8_t base;
      } u_warning_field2_free;
      u_warning_field2_free.real = this->warning_field2_free;
      *(outbuffer + offset + 0) = (u_warning_field2_free.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->warning_field2_free);
      union {
        bool real;
        uint8_t base;
      } u_contaminated;
      u_contaminated.real = this->contaminated;
      *(outbuffer + offset + 0) = (u_contaminated.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->contaminated);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      this->monitoring_case =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->monitoring_case);
      union {
        bool real;
        uint8_t base;
      } u_protective_field_free;
      u_protective_field_free.base = 0;
      u_protective_field_free.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->protective_field_free = u_protective_field_free.real;
      offset += sizeof(this->protective_field_free);
      union {
        bool real;
        uint8_t base;
      } u_warning_field1_free;
      u_warning_field1_free.base = 0;
      u_warning_field1_free.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->warning_field1_free = u_warning_field1_free.real;
      offset += sizeof(this->warning_field1_free);
      union {
        bool real;
        uint8_t base;
      } u_warning_field2_free;
      u_warning_field2_free.base = 0;
      u_warning_field2_free.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->warning_field2_free = u_warning_field2_free.real;
      offset += sizeof(this->warning_field2_free);
      union {
        bool real;
        uint8_t base;
      } u_contaminated;
      u_contaminated.base = 0;
      u_contaminated.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->contaminated = u_contaminated.real;
      offset += sizeof(this->contaminated);
     return offset;
    }

    virtual const char * getType() override { return "fetch_driver_msgs/SafetyLaserState"; };
    virtual const char * getMD5() override { return "3ba0845be197d9f4b6e251e9c720cc5e"; };

  };

}
#endif
