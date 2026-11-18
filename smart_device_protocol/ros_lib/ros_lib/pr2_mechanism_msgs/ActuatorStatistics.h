#ifndef _ROS_pr2_mechanism_msgs_ActuatorStatistics_h
#define _ROS_pr2_mechanism_msgs_ActuatorStatistics_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "ros/time.h"

namespace pr2_mechanism_msgs
{

  class ActuatorStatistics : public ros::Msg
  {
    public:
      typedef const char* _name_type;
      _name_type name;
      typedef int32_t _device_id_type;
      _device_id_type device_id;
      typedef ros::Time _timestamp_type;
      _timestamp_type timestamp;
      typedef int32_t _encoder_count_type;
      _encoder_count_type encoder_count;
      typedef float _encoder_offset_type;
      _encoder_offset_type encoder_offset;
      typedef float _position_type;
      _position_type position;
      typedef float _encoder_velocity_type;
      _encoder_velocity_type encoder_velocity;
      typedef float _velocity_type;
      _velocity_type velocity;
      typedef bool _calibration_reading_type;
      _calibration_reading_type calibration_reading;
      typedef bool _calibration_rising_edge_valid_type;
      _calibration_rising_edge_valid_type calibration_rising_edge_valid;
      typedef bool _calibration_falling_edge_valid_type;
      _calibration_falling_edge_valid_type calibration_falling_edge_valid;
      typedef float _last_calibration_rising_edge_type;
      _last_calibration_rising_edge_type last_calibration_rising_edge;
      typedef float _last_calibration_falling_edge_type;
      _last_calibration_falling_edge_type last_calibration_falling_edge;
      typedef bool _is_enabled_type;
      _is_enabled_type is_enabled;
      typedef bool _halted_type;
      _halted_type halted;
      typedef float _last_commanded_current_type;
      _last_commanded_current_type last_commanded_current;
      typedef float _last_commanded_effort_type;
      _last_commanded_effort_type last_commanded_effort;
      typedef float _last_executed_current_type;
      _last_executed_current_type last_executed_current;
      typedef float _last_executed_effort_type;
      _last_executed_effort_type last_executed_effort;
      typedef float _last_measured_current_type;
      _last_measured_current_type last_measured_current;
      typedef float _last_measured_effort_type;
      _last_measured_effort_type last_measured_effort;
      typedef float _motor_voltage_type;
      _motor_voltage_type motor_voltage;
      typedef int32_t _num_encoder_errors_type;
      _num_encoder_errors_type num_encoder_errors;

    ActuatorStatistics():
      name(""),
      device_id(0),
      timestamp(),
      encoder_count(0),
      encoder_offset(0),
      position(0),
      encoder_velocity(0),
      velocity(0),
      calibration_reading(0),
      calibration_rising_edge_valid(0),
      calibration_falling_edge_valid(0),
      last_calibration_rising_edge(0),
      last_calibration_falling_edge(0),
      is_enabled(0),
      halted(0),
      last_commanded_current(0),
      last_commanded_effort(0),
      last_executed_current(0),
      last_executed_effort(0),
      last_measured_current(0),
      last_measured_effort(0),
      motor_voltage(0),
      num_encoder_errors(0)
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
      } u_device_id;
      u_device_id.real = this->device_id;
      *(outbuffer + offset + 0) = (u_device_id.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_device_id.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_device_id.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_device_id.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->device_id);
      *(outbuffer + offset + 0) = (this->timestamp.sec >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->timestamp.sec >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->timestamp.sec >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->timestamp.sec >> (8 * 3)) & 0xFF;
      offset += sizeof(this->timestamp.sec);
      *(outbuffer + offset + 0) = (this->timestamp.nsec >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->timestamp.nsec >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->timestamp.nsec >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->timestamp.nsec >> (8 * 3)) & 0xFF;
      offset += sizeof(this->timestamp.nsec);
      union {
        int32_t real;
        uint32_t base;
      } u_encoder_count;
      u_encoder_count.real = this->encoder_count;
      *(outbuffer + offset + 0) = (u_encoder_count.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_encoder_count.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_encoder_count.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_encoder_count.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->encoder_count);
      offset += serializeAvrFloat64(outbuffer + offset, this->encoder_offset);
      offset += serializeAvrFloat64(outbuffer + offset, this->position);
      offset += serializeAvrFloat64(outbuffer + offset, this->encoder_velocity);
      offset += serializeAvrFloat64(outbuffer + offset, this->velocity);
      union {
        bool real;
        uint8_t base;
      } u_calibration_reading;
      u_calibration_reading.real = this->calibration_reading;
      *(outbuffer + offset + 0) = (u_calibration_reading.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->calibration_reading);
      union {
        bool real;
        uint8_t base;
      } u_calibration_rising_edge_valid;
      u_calibration_rising_edge_valid.real = this->calibration_rising_edge_valid;
      *(outbuffer + offset + 0) = (u_calibration_rising_edge_valid.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->calibration_rising_edge_valid);
      union {
        bool real;
        uint8_t base;
      } u_calibration_falling_edge_valid;
      u_calibration_falling_edge_valid.real = this->calibration_falling_edge_valid;
      *(outbuffer + offset + 0) = (u_calibration_falling_edge_valid.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->calibration_falling_edge_valid);
      offset += serializeAvrFloat64(outbuffer + offset, this->last_calibration_rising_edge);
      offset += serializeAvrFloat64(outbuffer + offset, this->last_calibration_falling_edge);
      union {
        bool real;
        uint8_t base;
      } u_is_enabled;
      u_is_enabled.real = this->is_enabled;
      *(outbuffer + offset + 0) = (u_is_enabled.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->is_enabled);
      union {
        bool real;
        uint8_t base;
      } u_halted;
      u_halted.real = this->halted;
      *(outbuffer + offset + 0) = (u_halted.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->halted);
      offset += serializeAvrFloat64(outbuffer + offset, this->last_commanded_current);
      offset += serializeAvrFloat64(outbuffer + offset, this->last_commanded_effort);
      offset += serializeAvrFloat64(outbuffer + offset, this->last_executed_current);
      offset += serializeAvrFloat64(outbuffer + offset, this->last_executed_effort);
      offset += serializeAvrFloat64(outbuffer + offset, this->last_measured_current);
      offset += serializeAvrFloat64(outbuffer + offset, this->last_measured_effort);
      offset += serializeAvrFloat64(outbuffer + offset, this->motor_voltage);
      union {
        int32_t real;
        uint32_t base;
      } u_num_encoder_errors;
      u_num_encoder_errors.real = this->num_encoder_errors;
      *(outbuffer + offset + 0) = (u_num_encoder_errors.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_num_encoder_errors.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_num_encoder_errors.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_num_encoder_errors.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->num_encoder_errors);
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
      } u_device_id;
      u_device_id.base = 0;
      u_device_id.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_device_id.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_device_id.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_device_id.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->device_id = u_device_id.real;
      offset += sizeof(this->device_id);
      this->timestamp.sec =  ((uint32_t) (*(inbuffer + offset)));
      this->timestamp.sec |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->timestamp.sec |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->timestamp.sec |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->timestamp.sec);
      this->timestamp.nsec =  ((uint32_t) (*(inbuffer + offset)));
      this->timestamp.nsec |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->timestamp.nsec |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->timestamp.nsec |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->timestamp.nsec);
      union {
        int32_t real;
        uint32_t base;
      } u_encoder_count;
      u_encoder_count.base = 0;
      u_encoder_count.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_encoder_count.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_encoder_count.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_encoder_count.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->encoder_count = u_encoder_count.real;
      offset += sizeof(this->encoder_count);
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->encoder_offset));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->position));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->encoder_velocity));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->velocity));
      union {
        bool real;
        uint8_t base;
      } u_calibration_reading;
      u_calibration_reading.base = 0;
      u_calibration_reading.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->calibration_reading = u_calibration_reading.real;
      offset += sizeof(this->calibration_reading);
      union {
        bool real;
        uint8_t base;
      } u_calibration_rising_edge_valid;
      u_calibration_rising_edge_valid.base = 0;
      u_calibration_rising_edge_valid.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->calibration_rising_edge_valid = u_calibration_rising_edge_valid.real;
      offset += sizeof(this->calibration_rising_edge_valid);
      union {
        bool real;
        uint8_t base;
      } u_calibration_falling_edge_valid;
      u_calibration_falling_edge_valid.base = 0;
      u_calibration_falling_edge_valid.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->calibration_falling_edge_valid = u_calibration_falling_edge_valid.real;
      offset += sizeof(this->calibration_falling_edge_valid);
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->last_calibration_rising_edge));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->last_calibration_falling_edge));
      union {
        bool real;
        uint8_t base;
      } u_is_enabled;
      u_is_enabled.base = 0;
      u_is_enabled.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->is_enabled = u_is_enabled.real;
      offset += sizeof(this->is_enabled);
      union {
        bool real;
        uint8_t base;
      } u_halted;
      u_halted.base = 0;
      u_halted.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->halted = u_halted.real;
      offset += sizeof(this->halted);
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->last_commanded_current));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->last_commanded_effort));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->last_executed_current));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->last_executed_effort));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->last_measured_current));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->last_measured_effort));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->motor_voltage));
      union {
        int32_t real;
        uint32_t base;
      } u_num_encoder_errors;
      u_num_encoder_errors.base = 0;
      u_num_encoder_errors.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_num_encoder_errors.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_num_encoder_errors.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_num_encoder_errors.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->num_encoder_errors = u_num_encoder_errors.real;
      offset += sizeof(this->num_encoder_errors);
     return offset;
    }

    virtual const char * getType() override { return "pr2_mechanism_msgs/ActuatorStatistics"; };
    virtual const char * getMD5() override { return "c37184273b29627de29382f1d3670175"; };

  };

}
#endif
