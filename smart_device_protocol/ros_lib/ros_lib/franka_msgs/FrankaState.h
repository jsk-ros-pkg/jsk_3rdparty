#ifndef _ROS_franka_msgs_FrankaState_h
#define _ROS_franka_msgs_FrankaState_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "franka_msgs/Errors.h"

namespace franka_msgs
{

  class FrankaState : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      float cartesian_collision[6];
      float cartesian_contact[6];
      float q[7];
      float q_d[7];
      float dq[7];
      float dq_d[7];
      float ddq_d[7];
      float theta[7];
      float dtheta[7];
      float tau_J[7];
      float dtau_J[7];
      float tau_J_d[7];
      float K_F_ext_hat_K[6];
      float elbow[2];
      float elbow_d[2];
      float elbow_c[2];
      float delbow_c[2];
      float ddelbow_c[2];
      float joint_collision[7];
      float joint_contact[7];
      float O_F_ext_hat_K[6];
      float O_dP_EE_d[6];
      float O_ddP_O[3];
      float O_dP_EE_c[6];
      float O_ddP_EE_c[6];
      float tau_ext_hat_filtered[7];
      typedef float _m_ee_type;
      _m_ee_type m_ee;
      float F_x_Cee[3];
      float I_ee[9];
      typedef float _m_load_type;
      _m_load_type m_load;
      float F_x_Cload[3];
      float I_load[9];
      typedef float _m_total_type;
      _m_total_type m_total;
      float F_x_Ctotal[3];
      float I_total[9];
      float O_T_EE[16];
      float O_T_EE_d[16];
      float O_T_EE_c[16];
      float F_T_EE[16];
      float F_T_NE[16];
      float NE_T_EE[16];
      float EE_T_K[16];
      typedef float _time_type;
      _time_type time;
      typedef float _control_command_success_rate_type;
      _control_command_success_rate_type control_command_success_rate;
      typedef uint8_t _robot_mode_type;
      _robot_mode_type robot_mode;
      typedef franka_msgs::Errors _current_errors_type;
      _current_errors_type current_errors;
      typedef franka_msgs::Errors _last_motion_errors_type;
      _last_motion_errors_type last_motion_errors;
      enum { ROBOT_MODE_OTHER = 0 };
      enum { ROBOT_MODE_IDLE = 1 };
      enum { ROBOT_MODE_MOVE = 2 };
      enum { ROBOT_MODE_GUIDING = 3 };
      enum { ROBOT_MODE_REFLEX = 4 };
      enum { ROBOT_MODE_USER_STOPPED = 5 };
      enum { ROBOT_MODE_AUTOMATIC_ERROR_RECOVERY = 6 };

    FrankaState():
      header(),
      cartesian_collision(),
      cartesian_contact(),
      q(),
      q_d(),
      dq(),
      dq_d(),
      ddq_d(),
      theta(),
      dtheta(),
      tau_J(),
      dtau_J(),
      tau_J_d(),
      K_F_ext_hat_K(),
      elbow(),
      elbow_d(),
      elbow_c(),
      delbow_c(),
      ddelbow_c(),
      joint_collision(),
      joint_contact(),
      O_F_ext_hat_K(),
      O_dP_EE_d(),
      O_ddP_O(),
      O_dP_EE_c(),
      O_ddP_EE_c(),
      tau_ext_hat_filtered(),
      m_ee(0),
      F_x_Cee(),
      I_ee(),
      m_load(0),
      F_x_Cload(),
      I_load(),
      m_total(0),
      F_x_Ctotal(),
      I_total(),
      O_T_EE(),
      O_T_EE_d(),
      O_T_EE_c(),
      F_T_EE(),
      F_T_NE(),
      NE_T_EE(),
      EE_T_K(),
      time(0),
      control_command_success_rate(0),
      robot_mode(0),
      current_errors(),
      last_motion_errors()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      for( uint32_t i = 0; i < 6; i++){
      offset += serializeAvrFloat64(outbuffer + offset, this->cartesian_collision[i]);
      }
      for( uint32_t i = 0; i < 6; i++){
      offset += serializeAvrFloat64(outbuffer + offset, this->cartesian_contact[i]);
      }
      for( uint32_t i = 0; i < 7; i++){
      offset += serializeAvrFloat64(outbuffer + offset, this->q[i]);
      }
      for( uint32_t i = 0; i < 7; i++){
      offset += serializeAvrFloat64(outbuffer + offset, this->q_d[i]);
      }
      for( uint32_t i = 0; i < 7; i++){
      offset += serializeAvrFloat64(outbuffer + offset, this->dq[i]);
      }
      for( uint32_t i = 0; i < 7; i++){
      offset += serializeAvrFloat64(outbuffer + offset, this->dq_d[i]);
      }
      for( uint32_t i = 0; i < 7; i++){
      offset += serializeAvrFloat64(outbuffer + offset, this->ddq_d[i]);
      }
      for( uint32_t i = 0; i < 7; i++){
      offset += serializeAvrFloat64(outbuffer + offset, this->theta[i]);
      }
      for( uint32_t i = 0; i < 7; i++){
      offset += serializeAvrFloat64(outbuffer + offset, this->dtheta[i]);
      }
      for( uint32_t i = 0; i < 7; i++){
      offset += serializeAvrFloat64(outbuffer + offset, this->tau_J[i]);
      }
      for( uint32_t i = 0; i < 7; i++){
      offset += serializeAvrFloat64(outbuffer + offset, this->dtau_J[i]);
      }
      for( uint32_t i = 0; i < 7; i++){
      offset += serializeAvrFloat64(outbuffer + offset, this->tau_J_d[i]);
      }
      for( uint32_t i = 0; i < 6; i++){
      offset += serializeAvrFloat64(outbuffer + offset, this->K_F_ext_hat_K[i]);
      }
      for( uint32_t i = 0; i < 2; i++){
      offset += serializeAvrFloat64(outbuffer + offset, this->elbow[i]);
      }
      for( uint32_t i = 0; i < 2; i++){
      offset += serializeAvrFloat64(outbuffer + offset, this->elbow_d[i]);
      }
      for( uint32_t i = 0; i < 2; i++){
      offset += serializeAvrFloat64(outbuffer + offset, this->elbow_c[i]);
      }
      for( uint32_t i = 0; i < 2; i++){
      offset += serializeAvrFloat64(outbuffer + offset, this->delbow_c[i]);
      }
      for( uint32_t i = 0; i < 2; i++){
      offset += serializeAvrFloat64(outbuffer + offset, this->ddelbow_c[i]);
      }
      for( uint32_t i = 0; i < 7; i++){
      offset += serializeAvrFloat64(outbuffer + offset, this->joint_collision[i]);
      }
      for( uint32_t i = 0; i < 7; i++){
      offset += serializeAvrFloat64(outbuffer + offset, this->joint_contact[i]);
      }
      for( uint32_t i = 0; i < 6; i++){
      offset += serializeAvrFloat64(outbuffer + offset, this->O_F_ext_hat_K[i]);
      }
      for( uint32_t i = 0; i < 6; i++){
      offset += serializeAvrFloat64(outbuffer + offset, this->O_dP_EE_d[i]);
      }
      for( uint32_t i = 0; i < 3; i++){
      offset += serializeAvrFloat64(outbuffer + offset, this->O_ddP_O[i]);
      }
      for( uint32_t i = 0; i < 6; i++){
      offset += serializeAvrFloat64(outbuffer + offset, this->O_dP_EE_c[i]);
      }
      for( uint32_t i = 0; i < 6; i++){
      offset += serializeAvrFloat64(outbuffer + offset, this->O_ddP_EE_c[i]);
      }
      for( uint32_t i = 0; i < 7; i++){
      offset += serializeAvrFloat64(outbuffer + offset, this->tau_ext_hat_filtered[i]);
      }
      offset += serializeAvrFloat64(outbuffer + offset, this->m_ee);
      for( uint32_t i = 0; i < 3; i++){
      offset += serializeAvrFloat64(outbuffer + offset, this->F_x_Cee[i]);
      }
      for( uint32_t i = 0; i < 9; i++){
      offset += serializeAvrFloat64(outbuffer + offset, this->I_ee[i]);
      }
      offset += serializeAvrFloat64(outbuffer + offset, this->m_load);
      for( uint32_t i = 0; i < 3; i++){
      offset += serializeAvrFloat64(outbuffer + offset, this->F_x_Cload[i]);
      }
      for( uint32_t i = 0; i < 9; i++){
      offset += serializeAvrFloat64(outbuffer + offset, this->I_load[i]);
      }
      offset += serializeAvrFloat64(outbuffer + offset, this->m_total);
      for( uint32_t i = 0; i < 3; i++){
      offset += serializeAvrFloat64(outbuffer + offset, this->F_x_Ctotal[i]);
      }
      for( uint32_t i = 0; i < 9; i++){
      offset += serializeAvrFloat64(outbuffer + offset, this->I_total[i]);
      }
      for( uint32_t i = 0; i < 16; i++){
      offset += serializeAvrFloat64(outbuffer + offset, this->O_T_EE[i]);
      }
      for( uint32_t i = 0; i < 16; i++){
      offset += serializeAvrFloat64(outbuffer + offset, this->O_T_EE_d[i]);
      }
      for( uint32_t i = 0; i < 16; i++){
      offset += serializeAvrFloat64(outbuffer + offset, this->O_T_EE_c[i]);
      }
      for( uint32_t i = 0; i < 16; i++){
      offset += serializeAvrFloat64(outbuffer + offset, this->F_T_EE[i]);
      }
      for( uint32_t i = 0; i < 16; i++){
      offset += serializeAvrFloat64(outbuffer + offset, this->F_T_NE[i]);
      }
      for( uint32_t i = 0; i < 16; i++){
      offset += serializeAvrFloat64(outbuffer + offset, this->NE_T_EE[i]);
      }
      for( uint32_t i = 0; i < 16; i++){
      offset += serializeAvrFloat64(outbuffer + offset, this->EE_T_K[i]);
      }
      offset += serializeAvrFloat64(outbuffer + offset, this->time);
      offset += serializeAvrFloat64(outbuffer + offset, this->control_command_success_rate);
      *(outbuffer + offset + 0) = (this->robot_mode >> (8 * 0)) & 0xFF;
      offset += sizeof(this->robot_mode);
      offset += this->current_errors.serialize(outbuffer + offset);
      offset += this->last_motion_errors.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      for( uint32_t i = 0; i < 6; i++){
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->cartesian_collision[i]));
      }
      for( uint32_t i = 0; i < 6; i++){
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->cartesian_contact[i]));
      }
      for( uint32_t i = 0; i < 7; i++){
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->q[i]));
      }
      for( uint32_t i = 0; i < 7; i++){
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->q_d[i]));
      }
      for( uint32_t i = 0; i < 7; i++){
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->dq[i]));
      }
      for( uint32_t i = 0; i < 7; i++){
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->dq_d[i]));
      }
      for( uint32_t i = 0; i < 7; i++){
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->ddq_d[i]));
      }
      for( uint32_t i = 0; i < 7; i++){
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->theta[i]));
      }
      for( uint32_t i = 0; i < 7; i++){
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->dtheta[i]));
      }
      for( uint32_t i = 0; i < 7; i++){
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->tau_J[i]));
      }
      for( uint32_t i = 0; i < 7; i++){
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->dtau_J[i]));
      }
      for( uint32_t i = 0; i < 7; i++){
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->tau_J_d[i]));
      }
      for( uint32_t i = 0; i < 6; i++){
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->K_F_ext_hat_K[i]));
      }
      for( uint32_t i = 0; i < 2; i++){
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->elbow[i]));
      }
      for( uint32_t i = 0; i < 2; i++){
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->elbow_d[i]));
      }
      for( uint32_t i = 0; i < 2; i++){
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->elbow_c[i]));
      }
      for( uint32_t i = 0; i < 2; i++){
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->delbow_c[i]));
      }
      for( uint32_t i = 0; i < 2; i++){
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->ddelbow_c[i]));
      }
      for( uint32_t i = 0; i < 7; i++){
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->joint_collision[i]));
      }
      for( uint32_t i = 0; i < 7; i++){
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->joint_contact[i]));
      }
      for( uint32_t i = 0; i < 6; i++){
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->O_F_ext_hat_K[i]));
      }
      for( uint32_t i = 0; i < 6; i++){
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->O_dP_EE_d[i]));
      }
      for( uint32_t i = 0; i < 3; i++){
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->O_ddP_O[i]));
      }
      for( uint32_t i = 0; i < 6; i++){
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->O_dP_EE_c[i]));
      }
      for( uint32_t i = 0; i < 6; i++){
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->O_ddP_EE_c[i]));
      }
      for( uint32_t i = 0; i < 7; i++){
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->tau_ext_hat_filtered[i]));
      }
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->m_ee));
      for( uint32_t i = 0; i < 3; i++){
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->F_x_Cee[i]));
      }
      for( uint32_t i = 0; i < 9; i++){
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->I_ee[i]));
      }
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->m_load));
      for( uint32_t i = 0; i < 3; i++){
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->F_x_Cload[i]));
      }
      for( uint32_t i = 0; i < 9; i++){
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->I_load[i]));
      }
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->m_total));
      for( uint32_t i = 0; i < 3; i++){
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->F_x_Ctotal[i]));
      }
      for( uint32_t i = 0; i < 9; i++){
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->I_total[i]));
      }
      for( uint32_t i = 0; i < 16; i++){
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->O_T_EE[i]));
      }
      for( uint32_t i = 0; i < 16; i++){
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->O_T_EE_d[i]));
      }
      for( uint32_t i = 0; i < 16; i++){
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->O_T_EE_c[i]));
      }
      for( uint32_t i = 0; i < 16; i++){
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->F_T_EE[i]));
      }
      for( uint32_t i = 0; i < 16; i++){
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->F_T_NE[i]));
      }
      for( uint32_t i = 0; i < 16; i++){
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->NE_T_EE[i]));
      }
      for( uint32_t i = 0; i < 16; i++){
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->EE_T_K[i]));
      }
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->time));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->control_command_success_rate));
      this->robot_mode =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->robot_mode);
      offset += this->current_errors.deserialize(inbuffer + offset);
      offset += this->last_motion_errors.deserialize(inbuffer + offset);
     return offset;
    }

    virtual const char * getType() override { return "franka_msgs/FrankaState"; };
    virtual const char * getMD5() override { return "431567d5df6caf4e4dd7385f25cb71ee"; };

  };

}
#endif
