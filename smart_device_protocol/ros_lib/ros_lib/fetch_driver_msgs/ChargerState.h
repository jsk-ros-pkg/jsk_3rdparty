#ifndef _ROS_fetch_driver_msgs_ChargerState_h
#define _ROS_fetch_driver_msgs_ChargerState_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace fetch_driver_msgs
{

  class ChargerState : public ros::Msg
  {
    public:
      typedef uint8_t _state_type;
      _state_type state;
      typedef uint8_t _error_type;
      _error_type error;
      typedef uint8_t _charging_mode_type;
      _charging_mode_type charging_mode;
      typedef uint8_t _limit_cause_type;
      _limit_cause_type limit_cause;
      typedef uint8_t _balancing_mode_type;
      _balancing_mode_type balancing_mode;
      typedef float _battery_voltage_type;
      _battery_voltage_type battery_voltage;
      typedef float _battery_half_voltage_type;
      _battery_half_voltage_type battery_half_voltage;
      typedef float _charger_voltage_type;
      _charger_voltage_type charger_voltage;
      typedef float _supply_voltage_type;
      _supply_voltage_type supply_voltage;
      typedef float _phase1_current_type;
      _phase1_current_type phase1_current;
      typedef float _phase2_current_type;
      _phase2_current_type phase2_current;
      typedef float _charger_temp_type;
      _charger_temp_type charger_temp;
      typedef float _battery_temp_type;
      _battery_temp_type battery_temp;
      typedef float _supply_connector_temp_type;
      _supply_connector_temp_type supply_connector_temp;
      typedef float _fan_speed_type;
      _fan_speed_type fan_speed;
      typedef float _battery_capacity_type;
      _battery_capacity_type battery_capacity;
      typedef float _battery_energy_type;
      _battery_energy_type battery_energy;
      typedef bool _is_charger_detected_type;
      _is_charger_detected_type is_charger_detected;

    ChargerState():
      state(0),
      error(0),
      charging_mode(0),
      limit_cause(0),
      balancing_mode(0),
      battery_voltage(0),
      battery_half_voltage(0),
      charger_voltage(0),
      supply_voltage(0),
      phase1_current(0),
      phase2_current(0),
      charger_temp(0),
      battery_temp(0),
      supply_connector_temp(0),
      fan_speed(0),
      battery_capacity(0),
      battery_energy(0),
      is_charger_detected(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->state >> (8 * 0)) & 0xFF;
      offset += sizeof(this->state);
      *(outbuffer + offset + 0) = (this->error >> (8 * 0)) & 0xFF;
      offset += sizeof(this->error);
      *(outbuffer + offset + 0) = (this->charging_mode >> (8 * 0)) & 0xFF;
      offset += sizeof(this->charging_mode);
      *(outbuffer + offset + 0) = (this->limit_cause >> (8 * 0)) & 0xFF;
      offset += sizeof(this->limit_cause);
      *(outbuffer + offset + 0) = (this->balancing_mode >> (8 * 0)) & 0xFF;
      offset += sizeof(this->balancing_mode);
      union {
        float real;
        uint32_t base;
      } u_battery_voltage;
      u_battery_voltage.real = this->battery_voltage;
      *(outbuffer + offset + 0) = (u_battery_voltage.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_battery_voltage.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_battery_voltage.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_battery_voltage.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->battery_voltage);
      union {
        float real;
        uint32_t base;
      } u_battery_half_voltage;
      u_battery_half_voltage.real = this->battery_half_voltage;
      *(outbuffer + offset + 0) = (u_battery_half_voltage.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_battery_half_voltage.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_battery_half_voltage.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_battery_half_voltage.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->battery_half_voltage);
      union {
        float real;
        uint32_t base;
      } u_charger_voltage;
      u_charger_voltage.real = this->charger_voltage;
      *(outbuffer + offset + 0) = (u_charger_voltage.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_charger_voltage.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_charger_voltage.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_charger_voltage.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->charger_voltage);
      union {
        float real;
        uint32_t base;
      } u_supply_voltage;
      u_supply_voltage.real = this->supply_voltage;
      *(outbuffer + offset + 0) = (u_supply_voltage.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_supply_voltage.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_supply_voltage.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_supply_voltage.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->supply_voltage);
      union {
        float real;
        uint32_t base;
      } u_phase1_current;
      u_phase1_current.real = this->phase1_current;
      *(outbuffer + offset + 0) = (u_phase1_current.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_phase1_current.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_phase1_current.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_phase1_current.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->phase1_current);
      union {
        float real;
        uint32_t base;
      } u_phase2_current;
      u_phase2_current.real = this->phase2_current;
      *(outbuffer + offset + 0) = (u_phase2_current.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_phase2_current.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_phase2_current.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_phase2_current.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->phase2_current);
      union {
        float real;
        uint32_t base;
      } u_charger_temp;
      u_charger_temp.real = this->charger_temp;
      *(outbuffer + offset + 0) = (u_charger_temp.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_charger_temp.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_charger_temp.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_charger_temp.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->charger_temp);
      union {
        float real;
        uint32_t base;
      } u_battery_temp;
      u_battery_temp.real = this->battery_temp;
      *(outbuffer + offset + 0) = (u_battery_temp.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_battery_temp.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_battery_temp.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_battery_temp.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->battery_temp);
      union {
        float real;
        uint32_t base;
      } u_supply_connector_temp;
      u_supply_connector_temp.real = this->supply_connector_temp;
      *(outbuffer + offset + 0) = (u_supply_connector_temp.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_supply_connector_temp.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_supply_connector_temp.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_supply_connector_temp.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->supply_connector_temp);
      union {
        float real;
        uint32_t base;
      } u_fan_speed;
      u_fan_speed.real = this->fan_speed;
      *(outbuffer + offset + 0) = (u_fan_speed.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_fan_speed.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_fan_speed.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_fan_speed.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->fan_speed);
      union {
        float real;
        uint32_t base;
      } u_battery_capacity;
      u_battery_capacity.real = this->battery_capacity;
      *(outbuffer + offset + 0) = (u_battery_capacity.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_battery_capacity.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_battery_capacity.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_battery_capacity.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->battery_capacity);
      union {
        float real;
        uint32_t base;
      } u_battery_energy;
      u_battery_energy.real = this->battery_energy;
      *(outbuffer + offset + 0) = (u_battery_energy.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_battery_energy.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_battery_energy.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_battery_energy.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->battery_energy);
      union {
        bool real;
        uint8_t base;
      } u_is_charger_detected;
      u_is_charger_detected.real = this->is_charger_detected;
      *(outbuffer + offset + 0) = (u_is_charger_detected.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->is_charger_detected);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      this->state =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->state);
      this->error =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->error);
      this->charging_mode =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->charging_mode);
      this->limit_cause =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->limit_cause);
      this->balancing_mode =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->balancing_mode);
      union {
        float real;
        uint32_t base;
      } u_battery_voltage;
      u_battery_voltage.base = 0;
      u_battery_voltage.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_battery_voltage.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_battery_voltage.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_battery_voltage.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->battery_voltage = u_battery_voltage.real;
      offset += sizeof(this->battery_voltage);
      union {
        float real;
        uint32_t base;
      } u_battery_half_voltage;
      u_battery_half_voltage.base = 0;
      u_battery_half_voltage.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_battery_half_voltage.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_battery_half_voltage.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_battery_half_voltage.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->battery_half_voltage = u_battery_half_voltage.real;
      offset += sizeof(this->battery_half_voltage);
      union {
        float real;
        uint32_t base;
      } u_charger_voltage;
      u_charger_voltage.base = 0;
      u_charger_voltage.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_charger_voltage.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_charger_voltage.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_charger_voltage.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->charger_voltage = u_charger_voltage.real;
      offset += sizeof(this->charger_voltage);
      union {
        float real;
        uint32_t base;
      } u_supply_voltage;
      u_supply_voltage.base = 0;
      u_supply_voltage.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_supply_voltage.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_supply_voltage.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_supply_voltage.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->supply_voltage = u_supply_voltage.real;
      offset += sizeof(this->supply_voltage);
      union {
        float real;
        uint32_t base;
      } u_phase1_current;
      u_phase1_current.base = 0;
      u_phase1_current.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_phase1_current.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_phase1_current.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_phase1_current.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->phase1_current = u_phase1_current.real;
      offset += sizeof(this->phase1_current);
      union {
        float real;
        uint32_t base;
      } u_phase2_current;
      u_phase2_current.base = 0;
      u_phase2_current.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_phase2_current.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_phase2_current.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_phase2_current.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->phase2_current = u_phase2_current.real;
      offset += sizeof(this->phase2_current);
      union {
        float real;
        uint32_t base;
      } u_charger_temp;
      u_charger_temp.base = 0;
      u_charger_temp.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_charger_temp.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_charger_temp.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_charger_temp.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->charger_temp = u_charger_temp.real;
      offset += sizeof(this->charger_temp);
      union {
        float real;
        uint32_t base;
      } u_battery_temp;
      u_battery_temp.base = 0;
      u_battery_temp.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_battery_temp.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_battery_temp.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_battery_temp.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->battery_temp = u_battery_temp.real;
      offset += sizeof(this->battery_temp);
      union {
        float real;
        uint32_t base;
      } u_supply_connector_temp;
      u_supply_connector_temp.base = 0;
      u_supply_connector_temp.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_supply_connector_temp.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_supply_connector_temp.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_supply_connector_temp.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->supply_connector_temp = u_supply_connector_temp.real;
      offset += sizeof(this->supply_connector_temp);
      union {
        float real;
        uint32_t base;
      } u_fan_speed;
      u_fan_speed.base = 0;
      u_fan_speed.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_fan_speed.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_fan_speed.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_fan_speed.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->fan_speed = u_fan_speed.real;
      offset += sizeof(this->fan_speed);
      union {
        float real;
        uint32_t base;
      } u_battery_capacity;
      u_battery_capacity.base = 0;
      u_battery_capacity.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_battery_capacity.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_battery_capacity.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_battery_capacity.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->battery_capacity = u_battery_capacity.real;
      offset += sizeof(this->battery_capacity);
      union {
        float real;
        uint32_t base;
      } u_battery_energy;
      u_battery_energy.base = 0;
      u_battery_energy.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_battery_energy.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_battery_energy.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_battery_energy.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->battery_energy = u_battery_energy.real;
      offset += sizeof(this->battery_energy);
      union {
        bool real;
        uint8_t base;
      } u_is_charger_detected;
      u_is_charger_detected.base = 0;
      u_is_charger_detected.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->is_charger_detected = u_is_charger_detected.real;
      offset += sizeof(this->is_charger_detected);
     return offset;
    }

    virtual const char * getType() override { return "fetch_driver_msgs/ChargerState"; };
    virtual const char * getMD5() override { return "d34eaaa0d5a93d36674702202ec485d9"; };

  };

}
#endif
