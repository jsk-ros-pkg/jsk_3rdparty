#ifndef _ROS_fetch_driver_msgs_BoardState_h
#define _ROS_fetch_driver_msgs_BoardState_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace fetch_driver_msgs
{

  class BoardState : public ros::Msg
  {
    public:
      typedef const char* _name_type;
      _name_type name;
      typedef int32_t _board_type_type;
      _board_type_type board_type;
      typedef int16_t _board_revision_type;
      _board_revision_type board_revision;
      typedef int16_t _software_revision_type;
      _software_revision_type software_revision;
      typedef int8_t _boot_status_type;
      _boot_status_type boot_status;
      typedef uint16_t _board_flags_type;
      _board_flags_type board_flags;
      typedef uint64_t _system_time_type;
      _system_time_type system_time;
      typedef float _voltage_type;
      _voltage_type voltage;
      typedef float _temperature_type;
      _temperature_type temperature;
      typedef uint32_t _packets_sent_type;
      _packets_sent_type packets_sent;
      typedef uint32_t _packets_recv_type;
      _packets_recv_type packets_recv;
      typedef uint32_t _packets_bad_type;
      _packets_bad_type packets_bad;
      typedef uint32_t _packets_back_type;
      _packets_back_type packets_back;

    BoardState():
      name(""),
      board_type(0),
      board_revision(0),
      software_revision(0),
      boot_status(0),
      board_flags(0),
      system_time(0),
      voltage(0),
      temperature(0),
      packets_sent(0),
      packets_recv(0),
      packets_bad(0),
      packets_back(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      uint32_t length_name = strlen(this->name);
      varToArr(outbuffer + offset, length_name);
      offset += 4;
      memcpy(outbuffer + offset, this->name, length_name);
      offset += length_name;
      union {
        int32_t real;
        uint32_t base;
      } u_board_type;
      u_board_type.real = this->board_type;
      *(outbuffer + offset + 0) = (u_board_type.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_board_type.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_board_type.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_board_type.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->board_type);
      union {
        int16_t real;
        uint16_t base;
      } u_board_revision;
      u_board_revision.real = this->board_revision;
      *(outbuffer + offset + 0) = (u_board_revision.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_board_revision.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->board_revision);
      union {
        int16_t real;
        uint16_t base;
      } u_software_revision;
      u_software_revision.real = this->software_revision;
      *(outbuffer + offset + 0) = (u_software_revision.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_software_revision.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->software_revision);
      union {
        int8_t real;
        uint8_t base;
      } u_boot_status;
      u_boot_status.real = this->boot_status;
      *(outbuffer + offset + 0) = (u_boot_status.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->boot_status);
      *(outbuffer + offset + 0) = (this->board_flags >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->board_flags >> (8 * 1)) & 0xFF;
      offset += sizeof(this->board_flags);
      *(outbuffer + offset + 0) = (this->system_time >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->system_time >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->system_time >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->system_time >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (this->system_time >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (this->system_time >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (this->system_time >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (this->system_time >> (8 * 7)) & 0xFF;
      offset += sizeof(this->system_time);
      union {
        float real;
        uint32_t base;
      } u_voltage;
      u_voltage.real = this->voltage;
      *(outbuffer + offset + 0) = (u_voltage.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_voltage.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_voltage.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_voltage.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->voltage);
      union {
        float real;
        uint32_t base;
      } u_temperature;
      u_temperature.real = this->temperature;
      *(outbuffer + offset + 0) = (u_temperature.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_temperature.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_temperature.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_temperature.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->temperature);
      *(outbuffer + offset + 0) = (this->packets_sent >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->packets_sent >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->packets_sent >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->packets_sent >> (8 * 3)) & 0xFF;
      offset += sizeof(this->packets_sent);
      *(outbuffer + offset + 0) = (this->packets_recv >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->packets_recv >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->packets_recv >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->packets_recv >> (8 * 3)) & 0xFF;
      offset += sizeof(this->packets_recv);
      *(outbuffer + offset + 0) = (this->packets_bad >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->packets_bad >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->packets_bad >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->packets_bad >> (8 * 3)) & 0xFF;
      offset += sizeof(this->packets_bad);
      *(outbuffer + offset + 0) = (this->packets_back >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->packets_back >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->packets_back >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->packets_back >> (8 * 3)) & 0xFF;
      offset += sizeof(this->packets_back);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      uint32_t length_name;
      arrToVar(length_name, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_name; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_name-1]=0;
      this->name = (char *)(inbuffer + offset-1);
      offset += length_name;
      union {
        int32_t real;
        uint32_t base;
      } u_board_type;
      u_board_type.base = 0;
      u_board_type.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_board_type.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_board_type.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_board_type.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->board_type = u_board_type.real;
      offset += sizeof(this->board_type);
      union {
        int16_t real;
        uint16_t base;
      } u_board_revision;
      u_board_revision.base = 0;
      u_board_revision.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_board_revision.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->board_revision = u_board_revision.real;
      offset += sizeof(this->board_revision);
      union {
        int16_t real;
        uint16_t base;
      } u_software_revision;
      u_software_revision.base = 0;
      u_software_revision.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_software_revision.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->software_revision = u_software_revision.real;
      offset += sizeof(this->software_revision);
      union {
        int8_t real;
        uint8_t base;
      } u_boot_status;
      u_boot_status.base = 0;
      u_boot_status.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->boot_status = u_boot_status.real;
      offset += sizeof(this->boot_status);
      this->board_flags =  ((uint16_t) (*(inbuffer + offset)));
      this->board_flags |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->board_flags);
      this->system_time =  ((uint64_t) (*(inbuffer + offset)));
      this->system_time |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->system_time |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->system_time |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->system_time |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      this->system_time |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      this->system_time |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      this->system_time |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      offset += sizeof(this->system_time);
      union {
        float real;
        uint32_t base;
      } u_voltage;
      u_voltage.base = 0;
      u_voltage.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_voltage.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_voltage.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_voltage.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->voltage = u_voltage.real;
      offset += sizeof(this->voltage);
      union {
        float real;
        uint32_t base;
      } u_temperature;
      u_temperature.base = 0;
      u_temperature.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_temperature.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_temperature.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_temperature.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->temperature = u_temperature.real;
      offset += sizeof(this->temperature);
      this->packets_sent =  ((uint32_t) (*(inbuffer + offset)));
      this->packets_sent |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->packets_sent |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->packets_sent |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->packets_sent);
      this->packets_recv =  ((uint32_t) (*(inbuffer + offset)));
      this->packets_recv |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->packets_recv |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->packets_recv |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->packets_recv);
      this->packets_bad =  ((uint32_t) (*(inbuffer + offset)));
      this->packets_bad |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->packets_bad |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->packets_bad |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->packets_bad);
      this->packets_back =  ((uint32_t) (*(inbuffer + offset)));
      this->packets_back |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->packets_back |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->packets_back |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->packets_back);
     return offset;
    }

    virtual const char * getType() override { return "fetch_driver_msgs/BoardState"; };
    virtual const char * getMD5() override { return "064739d3117a5b1aac96ad254216e299"; };

  };

}
#endif
