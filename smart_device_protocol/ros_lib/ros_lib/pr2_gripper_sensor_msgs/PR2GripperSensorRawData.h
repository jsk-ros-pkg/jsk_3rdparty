#ifndef _ROS_pr2_gripper_sensor_msgs_PR2GripperSensorRawData_h
#define _ROS_pr2_gripper_sensor_msgs_PR2GripperSensorRawData_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "ros/time.h"

namespace pr2_gripper_sensor_msgs
{

  class PR2GripperSensorRawData : public ros::Msg
  {
    public:
      typedef ros::Time _stamp_type;
      _stamp_type stamp;
      typedef float _left_finger_pad_force_type;
      _left_finger_pad_force_type left_finger_pad_force;
      typedef float _right_finger_pad_force_type;
      _right_finger_pad_force_type right_finger_pad_force;
      typedef float _left_finger_pad_force_filtered_type;
      _left_finger_pad_force_filtered_type left_finger_pad_force_filtered;
      typedef float _right_finger_pad_force_filtered_type;
      _right_finger_pad_force_filtered_type right_finger_pad_force_filtered;
      float left_finger_pad_forces[22];
      float right_finger_pad_forces[22];
      float left_finger_pad_forces_filtered[22];
      float right_finger_pad_forces_filtered[22];
      typedef float _acc_x_raw_type;
      _acc_x_raw_type acc_x_raw;
      typedef float _acc_y_raw_type;
      _acc_y_raw_type acc_y_raw;
      typedef float _acc_z_raw_type;
      _acc_z_raw_type acc_z_raw;
      typedef float _acc_x_filtered_type;
      _acc_x_filtered_type acc_x_filtered;
      typedef float _acc_y_filtered_type;
      _acc_y_filtered_type acc_y_filtered;
      typedef float _acc_z_filtered_type;
      _acc_z_filtered_type acc_z_filtered;
      typedef bool _left_contact_type;
      _left_contact_type left_contact;
      typedef bool _right_contact_type;
      _right_contact_type right_contact;

    PR2GripperSensorRawData():
      stamp(),
      left_finger_pad_force(0),
      right_finger_pad_force(0),
      left_finger_pad_force_filtered(0),
      right_finger_pad_force_filtered(0),
      left_finger_pad_forces(),
      right_finger_pad_forces(),
      left_finger_pad_forces_filtered(),
      right_finger_pad_forces_filtered(),
      acc_x_raw(0),
      acc_y_raw(0),
      acc_z_raw(0),
      acc_x_filtered(0),
      acc_y_filtered(0),
      acc_z_filtered(0),
      left_contact(0),
      right_contact(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->stamp.sec >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->stamp.sec >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->stamp.sec >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->stamp.sec >> (8 * 3)) & 0xFF;
      offset += sizeof(this->stamp.sec);
      *(outbuffer + offset + 0) = (this->stamp.nsec >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->stamp.nsec >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->stamp.nsec >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->stamp.nsec >> (8 * 3)) & 0xFF;
      offset += sizeof(this->stamp.nsec);
      offset += serializeAvrFloat64(outbuffer + offset, this->left_finger_pad_force);
      offset += serializeAvrFloat64(outbuffer + offset, this->right_finger_pad_force);
      offset += serializeAvrFloat64(outbuffer + offset, this->left_finger_pad_force_filtered);
      offset += serializeAvrFloat64(outbuffer + offset, this->right_finger_pad_force_filtered);
      for( uint32_t i = 0; i < 22; i++){
      offset += serializeAvrFloat64(outbuffer + offset, this->left_finger_pad_forces[i]);
      }
      for( uint32_t i = 0; i < 22; i++){
      offset += serializeAvrFloat64(outbuffer + offset, this->right_finger_pad_forces[i]);
      }
      for( uint32_t i = 0; i < 22; i++){
      offset += serializeAvrFloat64(outbuffer + offset, this->left_finger_pad_forces_filtered[i]);
      }
      for( uint32_t i = 0; i < 22; i++){
      offset += serializeAvrFloat64(outbuffer + offset, this->right_finger_pad_forces_filtered[i]);
      }
      offset += serializeAvrFloat64(outbuffer + offset, this->acc_x_raw);
      offset += serializeAvrFloat64(outbuffer + offset, this->acc_y_raw);
      offset += serializeAvrFloat64(outbuffer + offset, this->acc_z_raw);
      offset += serializeAvrFloat64(outbuffer + offset, this->acc_x_filtered);
      offset += serializeAvrFloat64(outbuffer + offset, this->acc_y_filtered);
      offset += serializeAvrFloat64(outbuffer + offset, this->acc_z_filtered);
      union {
        bool real;
        uint8_t base;
      } u_left_contact;
      u_left_contact.real = this->left_contact;
      *(outbuffer + offset + 0) = (u_left_contact.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->left_contact);
      union {
        bool real;
        uint8_t base;
      } u_right_contact;
      u_right_contact.real = this->right_contact;
      *(outbuffer + offset + 0) = (u_right_contact.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->right_contact);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      this->stamp.sec =  ((uint32_t) (*(inbuffer + offset)));
      this->stamp.sec |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->stamp.sec |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->stamp.sec |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->stamp.sec);
      this->stamp.nsec =  ((uint32_t) (*(inbuffer + offset)));
      this->stamp.nsec |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->stamp.nsec |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->stamp.nsec |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->stamp.nsec);
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->left_finger_pad_force));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->right_finger_pad_force));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->left_finger_pad_force_filtered));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->right_finger_pad_force_filtered));
      for( uint32_t i = 0; i < 22; i++){
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->left_finger_pad_forces[i]));
      }
      for( uint32_t i = 0; i < 22; i++){
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->right_finger_pad_forces[i]));
      }
      for( uint32_t i = 0; i < 22; i++){
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->left_finger_pad_forces_filtered[i]));
      }
      for( uint32_t i = 0; i < 22; i++){
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->right_finger_pad_forces_filtered[i]));
      }
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->acc_x_raw));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->acc_y_raw));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->acc_z_raw));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->acc_x_filtered));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->acc_y_filtered));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->acc_z_filtered));
      union {
        bool real;
        uint8_t base;
      } u_left_contact;
      u_left_contact.base = 0;
      u_left_contact.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->left_contact = u_left_contact.real;
      offset += sizeof(this->left_contact);
      union {
        bool real;
        uint8_t base;
      } u_right_contact;
      u_right_contact.base = 0;
      u_right_contact.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->right_contact = u_right_contact.real;
      offset += sizeof(this->right_contact);
     return offset;
    }

    virtual const char * getType() override { return "pr2_gripper_sensor_msgs/PR2GripperSensorRawData"; };
    virtual const char * getMD5() override { return "696a1f2e6969deb0bc6998636ae1b17e"; };

  };

}
#endif
