#ifndef _ROS_SERVICE_PowerBoardCommand2_h
#define _ROS_SERVICE_PowerBoardCommand2_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace pr2_power_board
{

static const char POWERBOARDCOMMAND2[] = "pr2_power_board/PowerBoardCommand2";

  class PowerBoardCommand2Request : public ros::Msg
  {
    public:
      typedef int32_t _circuit_type;
      _circuit_type circuit;
      typedef const char* _command_type;
      _command_type command;
      typedef bool _reset_stats_type;
      _reset_stats_type reset_stats;
      typedef bool _reset_circuits_type;
      _reset_circuits_type reset_circuits;
      enum { NUMBER_OF_CIRCUITS =  3 };
      enum { BASE =  0 };
      enum { RIGHT_ARM =  1 };
      enum { LEFT_ARM =  2 };

    PowerBoardCommand2Request():
      circuit(0),
      command(""),
      reset_stats(0),
      reset_circuits(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      union {
        int32_t real;
        uint32_t base;
      } u_circuit;
      u_circuit.real = this->circuit;
      *(outbuffer + offset + 0) = (u_circuit.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_circuit.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_circuit.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_circuit.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->circuit);
      uint32_t length_command = strlen(this->command);
      varToArr(outbuffer + offset, length_command);
      offset += 4;
      memcpy(outbuffer + offset, this->command, length_command);
      offset += length_command;
      union {
        bool real;
        uint8_t base;
      } u_reset_stats;
      u_reset_stats.real = this->reset_stats;
      *(outbuffer + offset + 0) = (u_reset_stats.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->reset_stats);
      union {
        bool real;
        uint8_t base;
      } u_reset_circuits;
      u_reset_circuits.real = this->reset_circuits;
      *(outbuffer + offset + 0) = (u_reset_circuits.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->reset_circuits);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      union {
        int32_t real;
        uint32_t base;
      } u_circuit;
      u_circuit.base = 0;
      u_circuit.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_circuit.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_circuit.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_circuit.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->circuit = u_circuit.real;
      offset += sizeof(this->circuit);
      uint32_t length_command;
      arrToVar(length_command, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_command; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_command-1]=0;
      this->command = (char *)(inbuffer + offset-1);
      offset += length_command;
      union {
        bool real;
        uint8_t base;
      } u_reset_stats;
      u_reset_stats.base = 0;
      u_reset_stats.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->reset_stats = u_reset_stats.real;
      offset += sizeof(this->reset_stats);
      union {
        bool real;
        uint8_t base;
      } u_reset_circuits;
      u_reset_circuits.base = 0;
      u_reset_circuits.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->reset_circuits = u_reset_circuits.real;
      offset += sizeof(this->reset_circuits);
     return offset;
    }

    virtual const char * getType() override { return POWERBOARDCOMMAND2; };
    virtual const char * getMD5() override { return "a31aee1db4294698375ff2a8cf53fea5"; };

  };

  class PowerBoardCommand2Response : public ros::Msg
  {
    public:
      typedef bool _success_type;
      _success_type success;

    PowerBoardCommand2Response():
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

    virtual const char * getType() override { return POWERBOARDCOMMAND2; };
    virtual const char * getMD5() override { return "358e233cde0c8a8bcfea4ce193f8fc15"; };

  };

  class PowerBoardCommand2 {
    public:
    typedef PowerBoardCommand2Request Request;
    typedef PowerBoardCommand2Response Response;
  };

}
#endif
