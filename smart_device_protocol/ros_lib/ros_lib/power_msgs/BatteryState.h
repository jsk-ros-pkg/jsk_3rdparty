#ifndef _ROS_power_msgs_BatteryState_h
#define _ROS_power_msgs_BatteryState_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "ros/duration.h"

namespace power_msgs
{

  class BatteryState : public ros::Msg
  {
    public:
      typedef const char* _name_type;
      _name_type name;
      typedef float _charge_level_type;
      _charge_level_type charge_level;
      typedef bool _is_charging_type;
      _is_charging_type is_charging;
      typedef ros::Duration _remaining_time_type;
      _remaining_time_type remaining_time;
      typedef float _total_capacity_type;
      _total_capacity_type total_capacity;
      typedef float _current_capacity_type;
      _current_capacity_type current_capacity;
      typedef float _battery_voltage_type;
      _battery_voltage_type battery_voltage;
      typedef float _supply_voltage_type;
      _supply_voltage_type supply_voltage;
      typedef float _charger_voltage_type;
      _charger_voltage_type charger_voltage;
      typedef bool _is_charger_detected_type;
      _is_charger_detected_type is_charger_detected;

    BatteryState():
      name(""),
      charge_level(0),
      is_charging(0),
      remaining_time(),
      total_capacity(0),
      current_capacity(0),
      battery_voltage(0),
      supply_voltage(0),
      charger_voltage(0),
      is_charger_detected(0)
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
        float real;
        uint32_t base;
      } u_charge_level;
      u_charge_level.real = this->charge_level;
      *(outbuffer + offset + 0) = (u_charge_level.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_charge_level.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_charge_level.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_charge_level.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->charge_level);
      union {
        bool real;
        uint8_t base;
      } u_is_charging;
      u_is_charging.real = this->is_charging;
      *(outbuffer + offset + 0) = (u_is_charging.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->is_charging);
      *(outbuffer + offset + 0) = (this->remaining_time.sec >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->remaining_time.sec >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->remaining_time.sec >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->remaining_time.sec >> (8 * 3)) & 0xFF;
      offset += sizeof(this->remaining_time.sec);
      *(outbuffer + offset + 0) = (this->remaining_time.nsec >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->remaining_time.nsec >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->remaining_time.nsec >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->remaining_time.nsec >> (8 * 3)) & 0xFF;
      offset += sizeof(this->remaining_time.nsec);
      union {
        float real;
        uint32_t base;
      } u_total_capacity;
      u_total_capacity.real = this->total_capacity;
      *(outbuffer + offset + 0) = (u_total_capacity.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_total_capacity.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_total_capacity.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_total_capacity.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->total_capacity);
      union {
        float real;
        uint32_t base;
      } u_current_capacity;
      u_current_capacity.real = this->current_capacity;
      *(outbuffer + offset + 0) = (u_current_capacity.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_current_capacity.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_current_capacity.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_current_capacity.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->current_capacity);
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
      } u_charger_voltage;
      u_charger_voltage.real = this->charger_voltage;
      *(outbuffer + offset + 0) = (u_charger_voltage.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_charger_voltage.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_charger_voltage.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_charger_voltage.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->charger_voltage);
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
        float real;
        uint32_t base;
      } u_charge_level;
      u_charge_level.base = 0;
      u_charge_level.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_charge_level.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_charge_level.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_charge_level.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->charge_level = u_charge_level.real;
      offset += sizeof(this->charge_level);
      union {
        bool real;
        uint8_t base;
      } u_is_charging;
      u_is_charging.base = 0;
      u_is_charging.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->is_charging = u_is_charging.real;
      offset += sizeof(this->is_charging);
      this->remaining_time.sec =  ((uint32_t) (*(inbuffer + offset)));
      this->remaining_time.sec |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->remaining_time.sec |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->remaining_time.sec |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->remaining_time.sec);
      this->remaining_time.nsec =  ((uint32_t) (*(inbuffer + offset)));
      this->remaining_time.nsec |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->remaining_time.nsec |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->remaining_time.nsec |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->remaining_time.nsec);
      union {
        float real;
        uint32_t base;
      } u_total_capacity;
      u_total_capacity.base = 0;
      u_total_capacity.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_total_capacity.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_total_capacity.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_total_capacity.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->total_capacity = u_total_capacity.real;
      offset += sizeof(this->total_capacity);
      union {
        float real;
        uint32_t base;
      } u_current_capacity;
      u_current_capacity.base = 0;
      u_current_capacity.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_current_capacity.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_current_capacity.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_current_capacity.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->current_capacity = u_current_capacity.real;
      offset += sizeof(this->current_capacity);
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
      } u_charger_voltage;
      u_charger_voltage.base = 0;
      u_charger_voltage.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_charger_voltage.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_charger_voltage.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_charger_voltage.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->charger_voltage = u_charger_voltage.real;
      offset += sizeof(this->charger_voltage);
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

    virtual const char * getType() override { return "power_msgs/BatteryState"; };
    virtual const char * getMD5() override { return "c7430ea7e5c95a5cba20f69647789404"; };

  };

}
#endif
