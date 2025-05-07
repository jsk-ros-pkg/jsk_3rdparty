#ifndef _ROS_franka_msgs_Errors_h
#define _ROS_franka_msgs_Errors_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace franka_msgs
{

  class Errors : public ros::Msg
  {
    public:
      typedef bool _joint_position_limits_violation_type;
      _joint_position_limits_violation_type joint_position_limits_violation;
      typedef bool _cartesian_position_limits_violation_type;
      _cartesian_position_limits_violation_type cartesian_position_limits_violation;
      typedef bool _self_collision_avoidance_violation_type;
      _self_collision_avoidance_violation_type self_collision_avoidance_violation;
      typedef bool _joint_velocity_violation_type;
      _joint_velocity_violation_type joint_velocity_violation;
      typedef bool _cartesian_velocity_violation_type;
      _cartesian_velocity_violation_type cartesian_velocity_violation;
      typedef bool _force_control_safety_violation_type;
      _force_control_safety_violation_type force_control_safety_violation;
      typedef bool _joint_reflex_type;
      _joint_reflex_type joint_reflex;
      typedef bool _cartesian_reflex_type;
      _cartesian_reflex_type cartesian_reflex;
      typedef bool _max_goal_pose_deviation_violation_type;
      _max_goal_pose_deviation_violation_type max_goal_pose_deviation_violation;
      typedef bool _max_path_pose_deviation_violation_type;
      _max_path_pose_deviation_violation_type max_path_pose_deviation_violation;
      typedef bool _cartesian_velocity_profile_safety_violation_type;
      _cartesian_velocity_profile_safety_violation_type cartesian_velocity_profile_safety_violation;
      typedef bool _joint_position_motion_generator_start_pose_invalid_type;
      _joint_position_motion_generator_start_pose_invalid_type joint_position_motion_generator_start_pose_invalid;
      typedef bool _joint_motion_generator_position_limits_violation_type;
      _joint_motion_generator_position_limits_violation_type joint_motion_generator_position_limits_violation;
      typedef bool _joint_motion_generator_velocity_limits_violation_type;
      _joint_motion_generator_velocity_limits_violation_type joint_motion_generator_velocity_limits_violation;
      typedef bool _joint_motion_generator_velocity_discontinuity_type;
      _joint_motion_generator_velocity_discontinuity_type joint_motion_generator_velocity_discontinuity;
      typedef bool _joint_motion_generator_acceleration_discontinuity_type;
      _joint_motion_generator_acceleration_discontinuity_type joint_motion_generator_acceleration_discontinuity;
      typedef bool _cartesian_position_motion_generator_start_pose_invalid_type;
      _cartesian_position_motion_generator_start_pose_invalid_type cartesian_position_motion_generator_start_pose_invalid;
      typedef bool _cartesian_motion_generator_elbow_limit_violation_type;
      _cartesian_motion_generator_elbow_limit_violation_type cartesian_motion_generator_elbow_limit_violation;
      typedef bool _cartesian_motion_generator_velocity_limits_violation_type;
      _cartesian_motion_generator_velocity_limits_violation_type cartesian_motion_generator_velocity_limits_violation;
      typedef bool _cartesian_motion_generator_velocity_discontinuity_type;
      _cartesian_motion_generator_velocity_discontinuity_type cartesian_motion_generator_velocity_discontinuity;
      typedef bool _cartesian_motion_generator_acceleration_discontinuity_type;
      _cartesian_motion_generator_acceleration_discontinuity_type cartesian_motion_generator_acceleration_discontinuity;
      typedef bool _cartesian_motion_generator_elbow_sign_inconsistent_type;
      _cartesian_motion_generator_elbow_sign_inconsistent_type cartesian_motion_generator_elbow_sign_inconsistent;
      typedef bool _cartesian_motion_generator_start_elbow_invalid_type;
      _cartesian_motion_generator_start_elbow_invalid_type cartesian_motion_generator_start_elbow_invalid;
      typedef bool _cartesian_motion_generator_joint_position_limits_violation_type;
      _cartesian_motion_generator_joint_position_limits_violation_type cartesian_motion_generator_joint_position_limits_violation;
      typedef bool _cartesian_motion_generator_joint_velocity_limits_violation_type;
      _cartesian_motion_generator_joint_velocity_limits_violation_type cartesian_motion_generator_joint_velocity_limits_violation;
      typedef bool _cartesian_motion_generator_joint_velocity_discontinuity_type;
      _cartesian_motion_generator_joint_velocity_discontinuity_type cartesian_motion_generator_joint_velocity_discontinuity;
      typedef bool _cartesian_motion_generator_joint_acceleration_discontinuity_type;
      _cartesian_motion_generator_joint_acceleration_discontinuity_type cartesian_motion_generator_joint_acceleration_discontinuity;
      typedef bool _cartesian_position_motion_generator_invalid_frame_type;
      _cartesian_position_motion_generator_invalid_frame_type cartesian_position_motion_generator_invalid_frame;
      typedef bool _force_controller_desired_force_tolerance_violation_type;
      _force_controller_desired_force_tolerance_violation_type force_controller_desired_force_tolerance_violation;
      typedef bool _controller_torque_discontinuity_type;
      _controller_torque_discontinuity_type controller_torque_discontinuity;
      typedef bool _start_elbow_sign_inconsistent_type;
      _start_elbow_sign_inconsistent_type start_elbow_sign_inconsistent;
      typedef bool _communication_constraints_violation_type;
      _communication_constraints_violation_type communication_constraints_violation;
      typedef bool _power_limit_violation_type;
      _power_limit_violation_type power_limit_violation;
      typedef bool _joint_p2p_insufficient_torque_for_planning_type;
      _joint_p2p_insufficient_torque_for_planning_type joint_p2p_insufficient_torque_for_planning;
      typedef bool _tau_j_range_violation_type;
      _tau_j_range_violation_type tau_j_range_violation;
      typedef bool _instability_detected_type;
      _instability_detected_type instability_detected;
      typedef bool _joint_move_in_wrong_direction_type;
      _joint_move_in_wrong_direction_type joint_move_in_wrong_direction;
      typedef bool _cartesian_spline_motion_generator_violation_type;
      _cartesian_spline_motion_generator_violation_type cartesian_spline_motion_generator_violation;
      typedef bool _joint_via_motion_generator_planning_joint_limit_violation_type;
      _joint_via_motion_generator_planning_joint_limit_violation_type joint_via_motion_generator_planning_joint_limit_violation;
      typedef bool _base_acceleration_initialization_timeout_type;
      _base_acceleration_initialization_timeout_type base_acceleration_initialization_timeout;
      typedef bool _base_acceleration_invalid_reading_type;
      _base_acceleration_invalid_reading_type base_acceleration_invalid_reading;

    Errors():
      joint_position_limits_violation(0),
      cartesian_position_limits_violation(0),
      self_collision_avoidance_violation(0),
      joint_velocity_violation(0),
      cartesian_velocity_violation(0),
      force_control_safety_violation(0),
      joint_reflex(0),
      cartesian_reflex(0),
      max_goal_pose_deviation_violation(0),
      max_path_pose_deviation_violation(0),
      cartesian_velocity_profile_safety_violation(0),
      joint_position_motion_generator_start_pose_invalid(0),
      joint_motion_generator_position_limits_violation(0),
      joint_motion_generator_velocity_limits_violation(0),
      joint_motion_generator_velocity_discontinuity(0),
      joint_motion_generator_acceleration_discontinuity(0),
      cartesian_position_motion_generator_start_pose_invalid(0),
      cartesian_motion_generator_elbow_limit_violation(0),
      cartesian_motion_generator_velocity_limits_violation(0),
      cartesian_motion_generator_velocity_discontinuity(0),
      cartesian_motion_generator_acceleration_discontinuity(0),
      cartesian_motion_generator_elbow_sign_inconsistent(0),
      cartesian_motion_generator_start_elbow_invalid(0),
      cartesian_motion_generator_joint_position_limits_violation(0),
      cartesian_motion_generator_joint_velocity_limits_violation(0),
      cartesian_motion_generator_joint_velocity_discontinuity(0),
      cartesian_motion_generator_joint_acceleration_discontinuity(0),
      cartesian_position_motion_generator_invalid_frame(0),
      force_controller_desired_force_tolerance_violation(0),
      controller_torque_discontinuity(0),
      start_elbow_sign_inconsistent(0),
      communication_constraints_violation(0),
      power_limit_violation(0),
      joint_p2p_insufficient_torque_for_planning(0),
      tau_j_range_violation(0),
      instability_detected(0),
      joint_move_in_wrong_direction(0),
      cartesian_spline_motion_generator_violation(0),
      joint_via_motion_generator_planning_joint_limit_violation(0),
      base_acceleration_initialization_timeout(0),
      base_acceleration_invalid_reading(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_joint_position_limits_violation;
      u_joint_position_limits_violation.real = this->joint_position_limits_violation;
      *(outbuffer + offset + 0) = (u_joint_position_limits_violation.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->joint_position_limits_violation);
      union {
        bool real;
        uint8_t base;
      } u_cartesian_position_limits_violation;
      u_cartesian_position_limits_violation.real = this->cartesian_position_limits_violation;
      *(outbuffer + offset + 0) = (u_cartesian_position_limits_violation.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->cartesian_position_limits_violation);
      union {
        bool real;
        uint8_t base;
      } u_self_collision_avoidance_violation;
      u_self_collision_avoidance_violation.real = this->self_collision_avoidance_violation;
      *(outbuffer + offset + 0) = (u_self_collision_avoidance_violation.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->self_collision_avoidance_violation);
      union {
        bool real;
        uint8_t base;
      } u_joint_velocity_violation;
      u_joint_velocity_violation.real = this->joint_velocity_violation;
      *(outbuffer + offset + 0) = (u_joint_velocity_violation.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->joint_velocity_violation);
      union {
        bool real;
        uint8_t base;
      } u_cartesian_velocity_violation;
      u_cartesian_velocity_violation.real = this->cartesian_velocity_violation;
      *(outbuffer + offset + 0) = (u_cartesian_velocity_violation.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->cartesian_velocity_violation);
      union {
        bool real;
        uint8_t base;
      } u_force_control_safety_violation;
      u_force_control_safety_violation.real = this->force_control_safety_violation;
      *(outbuffer + offset + 0) = (u_force_control_safety_violation.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->force_control_safety_violation);
      union {
        bool real;
        uint8_t base;
      } u_joint_reflex;
      u_joint_reflex.real = this->joint_reflex;
      *(outbuffer + offset + 0) = (u_joint_reflex.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->joint_reflex);
      union {
        bool real;
        uint8_t base;
      } u_cartesian_reflex;
      u_cartesian_reflex.real = this->cartesian_reflex;
      *(outbuffer + offset + 0) = (u_cartesian_reflex.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->cartesian_reflex);
      union {
        bool real;
        uint8_t base;
      } u_max_goal_pose_deviation_violation;
      u_max_goal_pose_deviation_violation.real = this->max_goal_pose_deviation_violation;
      *(outbuffer + offset + 0) = (u_max_goal_pose_deviation_violation.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->max_goal_pose_deviation_violation);
      union {
        bool real;
        uint8_t base;
      } u_max_path_pose_deviation_violation;
      u_max_path_pose_deviation_violation.real = this->max_path_pose_deviation_violation;
      *(outbuffer + offset + 0) = (u_max_path_pose_deviation_violation.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->max_path_pose_deviation_violation);
      union {
        bool real;
        uint8_t base;
      } u_cartesian_velocity_profile_safety_violation;
      u_cartesian_velocity_profile_safety_violation.real = this->cartesian_velocity_profile_safety_violation;
      *(outbuffer + offset + 0) = (u_cartesian_velocity_profile_safety_violation.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->cartesian_velocity_profile_safety_violation);
      union {
        bool real;
        uint8_t base;
      } u_joint_position_motion_generator_start_pose_invalid;
      u_joint_position_motion_generator_start_pose_invalid.real = this->joint_position_motion_generator_start_pose_invalid;
      *(outbuffer + offset + 0) = (u_joint_position_motion_generator_start_pose_invalid.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->joint_position_motion_generator_start_pose_invalid);
      union {
        bool real;
        uint8_t base;
      } u_joint_motion_generator_position_limits_violation;
      u_joint_motion_generator_position_limits_violation.real = this->joint_motion_generator_position_limits_violation;
      *(outbuffer + offset + 0) = (u_joint_motion_generator_position_limits_violation.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->joint_motion_generator_position_limits_violation);
      union {
        bool real;
        uint8_t base;
      } u_joint_motion_generator_velocity_limits_violation;
      u_joint_motion_generator_velocity_limits_violation.real = this->joint_motion_generator_velocity_limits_violation;
      *(outbuffer + offset + 0) = (u_joint_motion_generator_velocity_limits_violation.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->joint_motion_generator_velocity_limits_violation);
      union {
        bool real;
        uint8_t base;
      } u_joint_motion_generator_velocity_discontinuity;
      u_joint_motion_generator_velocity_discontinuity.real = this->joint_motion_generator_velocity_discontinuity;
      *(outbuffer + offset + 0) = (u_joint_motion_generator_velocity_discontinuity.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->joint_motion_generator_velocity_discontinuity);
      union {
        bool real;
        uint8_t base;
      } u_joint_motion_generator_acceleration_discontinuity;
      u_joint_motion_generator_acceleration_discontinuity.real = this->joint_motion_generator_acceleration_discontinuity;
      *(outbuffer + offset + 0) = (u_joint_motion_generator_acceleration_discontinuity.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->joint_motion_generator_acceleration_discontinuity);
      union {
        bool real;
        uint8_t base;
      } u_cartesian_position_motion_generator_start_pose_invalid;
      u_cartesian_position_motion_generator_start_pose_invalid.real = this->cartesian_position_motion_generator_start_pose_invalid;
      *(outbuffer + offset + 0) = (u_cartesian_position_motion_generator_start_pose_invalid.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->cartesian_position_motion_generator_start_pose_invalid);
      union {
        bool real;
        uint8_t base;
      } u_cartesian_motion_generator_elbow_limit_violation;
      u_cartesian_motion_generator_elbow_limit_violation.real = this->cartesian_motion_generator_elbow_limit_violation;
      *(outbuffer + offset + 0) = (u_cartesian_motion_generator_elbow_limit_violation.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->cartesian_motion_generator_elbow_limit_violation);
      union {
        bool real;
        uint8_t base;
      } u_cartesian_motion_generator_velocity_limits_violation;
      u_cartesian_motion_generator_velocity_limits_violation.real = this->cartesian_motion_generator_velocity_limits_violation;
      *(outbuffer + offset + 0) = (u_cartesian_motion_generator_velocity_limits_violation.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->cartesian_motion_generator_velocity_limits_violation);
      union {
        bool real;
        uint8_t base;
      } u_cartesian_motion_generator_velocity_discontinuity;
      u_cartesian_motion_generator_velocity_discontinuity.real = this->cartesian_motion_generator_velocity_discontinuity;
      *(outbuffer + offset + 0) = (u_cartesian_motion_generator_velocity_discontinuity.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->cartesian_motion_generator_velocity_discontinuity);
      union {
        bool real;
        uint8_t base;
      } u_cartesian_motion_generator_acceleration_discontinuity;
      u_cartesian_motion_generator_acceleration_discontinuity.real = this->cartesian_motion_generator_acceleration_discontinuity;
      *(outbuffer + offset + 0) = (u_cartesian_motion_generator_acceleration_discontinuity.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->cartesian_motion_generator_acceleration_discontinuity);
      union {
        bool real;
        uint8_t base;
      } u_cartesian_motion_generator_elbow_sign_inconsistent;
      u_cartesian_motion_generator_elbow_sign_inconsistent.real = this->cartesian_motion_generator_elbow_sign_inconsistent;
      *(outbuffer + offset + 0) = (u_cartesian_motion_generator_elbow_sign_inconsistent.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->cartesian_motion_generator_elbow_sign_inconsistent);
      union {
        bool real;
        uint8_t base;
      } u_cartesian_motion_generator_start_elbow_invalid;
      u_cartesian_motion_generator_start_elbow_invalid.real = this->cartesian_motion_generator_start_elbow_invalid;
      *(outbuffer + offset + 0) = (u_cartesian_motion_generator_start_elbow_invalid.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->cartesian_motion_generator_start_elbow_invalid);
      union {
        bool real;
        uint8_t base;
      } u_cartesian_motion_generator_joint_position_limits_violation;
      u_cartesian_motion_generator_joint_position_limits_violation.real = this->cartesian_motion_generator_joint_position_limits_violation;
      *(outbuffer + offset + 0) = (u_cartesian_motion_generator_joint_position_limits_violation.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->cartesian_motion_generator_joint_position_limits_violation);
      union {
        bool real;
        uint8_t base;
      } u_cartesian_motion_generator_joint_velocity_limits_violation;
      u_cartesian_motion_generator_joint_velocity_limits_violation.real = this->cartesian_motion_generator_joint_velocity_limits_violation;
      *(outbuffer + offset + 0) = (u_cartesian_motion_generator_joint_velocity_limits_violation.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->cartesian_motion_generator_joint_velocity_limits_violation);
      union {
        bool real;
        uint8_t base;
      } u_cartesian_motion_generator_joint_velocity_discontinuity;
      u_cartesian_motion_generator_joint_velocity_discontinuity.real = this->cartesian_motion_generator_joint_velocity_discontinuity;
      *(outbuffer + offset + 0) = (u_cartesian_motion_generator_joint_velocity_discontinuity.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->cartesian_motion_generator_joint_velocity_discontinuity);
      union {
        bool real;
        uint8_t base;
      } u_cartesian_motion_generator_joint_acceleration_discontinuity;
      u_cartesian_motion_generator_joint_acceleration_discontinuity.real = this->cartesian_motion_generator_joint_acceleration_discontinuity;
      *(outbuffer + offset + 0) = (u_cartesian_motion_generator_joint_acceleration_discontinuity.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->cartesian_motion_generator_joint_acceleration_discontinuity);
      union {
        bool real;
        uint8_t base;
      } u_cartesian_position_motion_generator_invalid_frame;
      u_cartesian_position_motion_generator_invalid_frame.real = this->cartesian_position_motion_generator_invalid_frame;
      *(outbuffer + offset + 0) = (u_cartesian_position_motion_generator_invalid_frame.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->cartesian_position_motion_generator_invalid_frame);
      union {
        bool real;
        uint8_t base;
      } u_force_controller_desired_force_tolerance_violation;
      u_force_controller_desired_force_tolerance_violation.real = this->force_controller_desired_force_tolerance_violation;
      *(outbuffer + offset + 0) = (u_force_controller_desired_force_tolerance_violation.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->force_controller_desired_force_tolerance_violation);
      union {
        bool real;
        uint8_t base;
      } u_controller_torque_discontinuity;
      u_controller_torque_discontinuity.real = this->controller_torque_discontinuity;
      *(outbuffer + offset + 0) = (u_controller_torque_discontinuity.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->controller_torque_discontinuity);
      union {
        bool real;
        uint8_t base;
      } u_start_elbow_sign_inconsistent;
      u_start_elbow_sign_inconsistent.real = this->start_elbow_sign_inconsistent;
      *(outbuffer + offset + 0) = (u_start_elbow_sign_inconsistent.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->start_elbow_sign_inconsistent);
      union {
        bool real;
        uint8_t base;
      } u_communication_constraints_violation;
      u_communication_constraints_violation.real = this->communication_constraints_violation;
      *(outbuffer + offset + 0) = (u_communication_constraints_violation.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->communication_constraints_violation);
      union {
        bool real;
        uint8_t base;
      } u_power_limit_violation;
      u_power_limit_violation.real = this->power_limit_violation;
      *(outbuffer + offset + 0) = (u_power_limit_violation.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->power_limit_violation);
      union {
        bool real;
        uint8_t base;
      } u_joint_p2p_insufficient_torque_for_planning;
      u_joint_p2p_insufficient_torque_for_planning.real = this->joint_p2p_insufficient_torque_for_planning;
      *(outbuffer + offset + 0) = (u_joint_p2p_insufficient_torque_for_planning.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->joint_p2p_insufficient_torque_for_planning);
      union {
        bool real;
        uint8_t base;
      } u_tau_j_range_violation;
      u_tau_j_range_violation.real = this->tau_j_range_violation;
      *(outbuffer + offset + 0) = (u_tau_j_range_violation.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->tau_j_range_violation);
      union {
        bool real;
        uint8_t base;
      } u_instability_detected;
      u_instability_detected.real = this->instability_detected;
      *(outbuffer + offset + 0) = (u_instability_detected.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->instability_detected);
      union {
        bool real;
        uint8_t base;
      } u_joint_move_in_wrong_direction;
      u_joint_move_in_wrong_direction.real = this->joint_move_in_wrong_direction;
      *(outbuffer + offset + 0) = (u_joint_move_in_wrong_direction.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->joint_move_in_wrong_direction);
      union {
        bool real;
        uint8_t base;
      } u_cartesian_spline_motion_generator_violation;
      u_cartesian_spline_motion_generator_violation.real = this->cartesian_spline_motion_generator_violation;
      *(outbuffer + offset + 0) = (u_cartesian_spline_motion_generator_violation.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->cartesian_spline_motion_generator_violation);
      union {
        bool real;
        uint8_t base;
      } u_joint_via_motion_generator_planning_joint_limit_violation;
      u_joint_via_motion_generator_planning_joint_limit_violation.real = this->joint_via_motion_generator_planning_joint_limit_violation;
      *(outbuffer + offset + 0) = (u_joint_via_motion_generator_planning_joint_limit_violation.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->joint_via_motion_generator_planning_joint_limit_violation);
      union {
        bool real;
        uint8_t base;
      } u_base_acceleration_initialization_timeout;
      u_base_acceleration_initialization_timeout.real = this->base_acceleration_initialization_timeout;
      *(outbuffer + offset + 0) = (u_base_acceleration_initialization_timeout.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->base_acceleration_initialization_timeout);
      union {
        bool real;
        uint8_t base;
      } u_base_acceleration_invalid_reading;
      u_base_acceleration_invalid_reading.real = this->base_acceleration_invalid_reading;
      *(outbuffer + offset + 0) = (u_base_acceleration_invalid_reading.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->base_acceleration_invalid_reading);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_joint_position_limits_violation;
      u_joint_position_limits_violation.base = 0;
      u_joint_position_limits_violation.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->joint_position_limits_violation = u_joint_position_limits_violation.real;
      offset += sizeof(this->joint_position_limits_violation);
      union {
        bool real;
        uint8_t base;
      } u_cartesian_position_limits_violation;
      u_cartesian_position_limits_violation.base = 0;
      u_cartesian_position_limits_violation.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->cartesian_position_limits_violation = u_cartesian_position_limits_violation.real;
      offset += sizeof(this->cartesian_position_limits_violation);
      union {
        bool real;
        uint8_t base;
      } u_self_collision_avoidance_violation;
      u_self_collision_avoidance_violation.base = 0;
      u_self_collision_avoidance_violation.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->self_collision_avoidance_violation = u_self_collision_avoidance_violation.real;
      offset += sizeof(this->self_collision_avoidance_violation);
      union {
        bool real;
        uint8_t base;
      } u_joint_velocity_violation;
      u_joint_velocity_violation.base = 0;
      u_joint_velocity_violation.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->joint_velocity_violation = u_joint_velocity_violation.real;
      offset += sizeof(this->joint_velocity_violation);
      union {
        bool real;
        uint8_t base;
      } u_cartesian_velocity_violation;
      u_cartesian_velocity_violation.base = 0;
      u_cartesian_velocity_violation.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->cartesian_velocity_violation = u_cartesian_velocity_violation.real;
      offset += sizeof(this->cartesian_velocity_violation);
      union {
        bool real;
        uint8_t base;
      } u_force_control_safety_violation;
      u_force_control_safety_violation.base = 0;
      u_force_control_safety_violation.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->force_control_safety_violation = u_force_control_safety_violation.real;
      offset += sizeof(this->force_control_safety_violation);
      union {
        bool real;
        uint8_t base;
      } u_joint_reflex;
      u_joint_reflex.base = 0;
      u_joint_reflex.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->joint_reflex = u_joint_reflex.real;
      offset += sizeof(this->joint_reflex);
      union {
        bool real;
        uint8_t base;
      } u_cartesian_reflex;
      u_cartesian_reflex.base = 0;
      u_cartesian_reflex.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->cartesian_reflex = u_cartesian_reflex.real;
      offset += sizeof(this->cartesian_reflex);
      union {
        bool real;
        uint8_t base;
      } u_max_goal_pose_deviation_violation;
      u_max_goal_pose_deviation_violation.base = 0;
      u_max_goal_pose_deviation_violation.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->max_goal_pose_deviation_violation = u_max_goal_pose_deviation_violation.real;
      offset += sizeof(this->max_goal_pose_deviation_violation);
      union {
        bool real;
        uint8_t base;
      } u_max_path_pose_deviation_violation;
      u_max_path_pose_deviation_violation.base = 0;
      u_max_path_pose_deviation_violation.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->max_path_pose_deviation_violation = u_max_path_pose_deviation_violation.real;
      offset += sizeof(this->max_path_pose_deviation_violation);
      union {
        bool real;
        uint8_t base;
      } u_cartesian_velocity_profile_safety_violation;
      u_cartesian_velocity_profile_safety_violation.base = 0;
      u_cartesian_velocity_profile_safety_violation.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->cartesian_velocity_profile_safety_violation = u_cartesian_velocity_profile_safety_violation.real;
      offset += sizeof(this->cartesian_velocity_profile_safety_violation);
      union {
        bool real;
        uint8_t base;
      } u_joint_position_motion_generator_start_pose_invalid;
      u_joint_position_motion_generator_start_pose_invalid.base = 0;
      u_joint_position_motion_generator_start_pose_invalid.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->joint_position_motion_generator_start_pose_invalid = u_joint_position_motion_generator_start_pose_invalid.real;
      offset += sizeof(this->joint_position_motion_generator_start_pose_invalid);
      union {
        bool real;
        uint8_t base;
      } u_joint_motion_generator_position_limits_violation;
      u_joint_motion_generator_position_limits_violation.base = 0;
      u_joint_motion_generator_position_limits_violation.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->joint_motion_generator_position_limits_violation = u_joint_motion_generator_position_limits_violation.real;
      offset += sizeof(this->joint_motion_generator_position_limits_violation);
      union {
        bool real;
        uint8_t base;
      } u_joint_motion_generator_velocity_limits_violation;
      u_joint_motion_generator_velocity_limits_violation.base = 0;
      u_joint_motion_generator_velocity_limits_violation.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->joint_motion_generator_velocity_limits_violation = u_joint_motion_generator_velocity_limits_violation.real;
      offset += sizeof(this->joint_motion_generator_velocity_limits_violation);
      union {
        bool real;
        uint8_t base;
      } u_joint_motion_generator_velocity_discontinuity;
      u_joint_motion_generator_velocity_discontinuity.base = 0;
      u_joint_motion_generator_velocity_discontinuity.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->joint_motion_generator_velocity_discontinuity = u_joint_motion_generator_velocity_discontinuity.real;
      offset += sizeof(this->joint_motion_generator_velocity_discontinuity);
      union {
        bool real;
        uint8_t base;
      } u_joint_motion_generator_acceleration_discontinuity;
      u_joint_motion_generator_acceleration_discontinuity.base = 0;
      u_joint_motion_generator_acceleration_discontinuity.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->joint_motion_generator_acceleration_discontinuity = u_joint_motion_generator_acceleration_discontinuity.real;
      offset += sizeof(this->joint_motion_generator_acceleration_discontinuity);
      union {
        bool real;
        uint8_t base;
      } u_cartesian_position_motion_generator_start_pose_invalid;
      u_cartesian_position_motion_generator_start_pose_invalid.base = 0;
      u_cartesian_position_motion_generator_start_pose_invalid.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->cartesian_position_motion_generator_start_pose_invalid = u_cartesian_position_motion_generator_start_pose_invalid.real;
      offset += sizeof(this->cartesian_position_motion_generator_start_pose_invalid);
      union {
        bool real;
        uint8_t base;
      } u_cartesian_motion_generator_elbow_limit_violation;
      u_cartesian_motion_generator_elbow_limit_violation.base = 0;
      u_cartesian_motion_generator_elbow_limit_violation.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->cartesian_motion_generator_elbow_limit_violation = u_cartesian_motion_generator_elbow_limit_violation.real;
      offset += sizeof(this->cartesian_motion_generator_elbow_limit_violation);
      union {
        bool real;
        uint8_t base;
      } u_cartesian_motion_generator_velocity_limits_violation;
      u_cartesian_motion_generator_velocity_limits_violation.base = 0;
      u_cartesian_motion_generator_velocity_limits_violation.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->cartesian_motion_generator_velocity_limits_violation = u_cartesian_motion_generator_velocity_limits_violation.real;
      offset += sizeof(this->cartesian_motion_generator_velocity_limits_violation);
      union {
        bool real;
        uint8_t base;
      } u_cartesian_motion_generator_velocity_discontinuity;
      u_cartesian_motion_generator_velocity_discontinuity.base = 0;
      u_cartesian_motion_generator_velocity_discontinuity.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->cartesian_motion_generator_velocity_discontinuity = u_cartesian_motion_generator_velocity_discontinuity.real;
      offset += sizeof(this->cartesian_motion_generator_velocity_discontinuity);
      union {
        bool real;
        uint8_t base;
      } u_cartesian_motion_generator_acceleration_discontinuity;
      u_cartesian_motion_generator_acceleration_discontinuity.base = 0;
      u_cartesian_motion_generator_acceleration_discontinuity.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->cartesian_motion_generator_acceleration_discontinuity = u_cartesian_motion_generator_acceleration_discontinuity.real;
      offset += sizeof(this->cartesian_motion_generator_acceleration_discontinuity);
      union {
        bool real;
        uint8_t base;
      } u_cartesian_motion_generator_elbow_sign_inconsistent;
      u_cartesian_motion_generator_elbow_sign_inconsistent.base = 0;
      u_cartesian_motion_generator_elbow_sign_inconsistent.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->cartesian_motion_generator_elbow_sign_inconsistent = u_cartesian_motion_generator_elbow_sign_inconsistent.real;
      offset += sizeof(this->cartesian_motion_generator_elbow_sign_inconsistent);
      union {
        bool real;
        uint8_t base;
      } u_cartesian_motion_generator_start_elbow_invalid;
      u_cartesian_motion_generator_start_elbow_invalid.base = 0;
      u_cartesian_motion_generator_start_elbow_invalid.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->cartesian_motion_generator_start_elbow_invalid = u_cartesian_motion_generator_start_elbow_invalid.real;
      offset += sizeof(this->cartesian_motion_generator_start_elbow_invalid);
      union {
        bool real;
        uint8_t base;
      } u_cartesian_motion_generator_joint_position_limits_violation;
      u_cartesian_motion_generator_joint_position_limits_violation.base = 0;
      u_cartesian_motion_generator_joint_position_limits_violation.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->cartesian_motion_generator_joint_position_limits_violation = u_cartesian_motion_generator_joint_position_limits_violation.real;
      offset += sizeof(this->cartesian_motion_generator_joint_position_limits_violation);
      union {
        bool real;
        uint8_t base;
      } u_cartesian_motion_generator_joint_velocity_limits_violation;
      u_cartesian_motion_generator_joint_velocity_limits_violation.base = 0;
      u_cartesian_motion_generator_joint_velocity_limits_violation.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->cartesian_motion_generator_joint_velocity_limits_violation = u_cartesian_motion_generator_joint_velocity_limits_violation.real;
      offset += sizeof(this->cartesian_motion_generator_joint_velocity_limits_violation);
      union {
        bool real;
        uint8_t base;
      } u_cartesian_motion_generator_joint_velocity_discontinuity;
      u_cartesian_motion_generator_joint_velocity_discontinuity.base = 0;
      u_cartesian_motion_generator_joint_velocity_discontinuity.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->cartesian_motion_generator_joint_velocity_discontinuity = u_cartesian_motion_generator_joint_velocity_discontinuity.real;
      offset += sizeof(this->cartesian_motion_generator_joint_velocity_discontinuity);
      union {
        bool real;
        uint8_t base;
      } u_cartesian_motion_generator_joint_acceleration_discontinuity;
      u_cartesian_motion_generator_joint_acceleration_discontinuity.base = 0;
      u_cartesian_motion_generator_joint_acceleration_discontinuity.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->cartesian_motion_generator_joint_acceleration_discontinuity = u_cartesian_motion_generator_joint_acceleration_discontinuity.real;
      offset += sizeof(this->cartesian_motion_generator_joint_acceleration_discontinuity);
      union {
        bool real;
        uint8_t base;
      } u_cartesian_position_motion_generator_invalid_frame;
      u_cartesian_position_motion_generator_invalid_frame.base = 0;
      u_cartesian_position_motion_generator_invalid_frame.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->cartesian_position_motion_generator_invalid_frame = u_cartesian_position_motion_generator_invalid_frame.real;
      offset += sizeof(this->cartesian_position_motion_generator_invalid_frame);
      union {
        bool real;
        uint8_t base;
      } u_force_controller_desired_force_tolerance_violation;
      u_force_controller_desired_force_tolerance_violation.base = 0;
      u_force_controller_desired_force_tolerance_violation.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->force_controller_desired_force_tolerance_violation = u_force_controller_desired_force_tolerance_violation.real;
      offset += sizeof(this->force_controller_desired_force_tolerance_violation);
      union {
        bool real;
        uint8_t base;
      } u_controller_torque_discontinuity;
      u_controller_torque_discontinuity.base = 0;
      u_controller_torque_discontinuity.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->controller_torque_discontinuity = u_controller_torque_discontinuity.real;
      offset += sizeof(this->controller_torque_discontinuity);
      union {
        bool real;
        uint8_t base;
      } u_start_elbow_sign_inconsistent;
      u_start_elbow_sign_inconsistent.base = 0;
      u_start_elbow_sign_inconsistent.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->start_elbow_sign_inconsistent = u_start_elbow_sign_inconsistent.real;
      offset += sizeof(this->start_elbow_sign_inconsistent);
      union {
        bool real;
        uint8_t base;
      } u_communication_constraints_violation;
      u_communication_constraints_violation.base = 0;
      u_communication_constraints_violation.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->communication_constraints_violation = u_communication_constraints_violation.real;
      offset += sizeof(this->communication_constraints_violation);
      union {
        bool real;
        uint8_t base;
      } u_power_limit_violation;
      u_power_limit_violation.base = 0;
      u_power_limit_violation.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->power_limit_violation = u_power_limit_violation.real;
      offset += sizeof(this->power_limit_violation);
      union {
        bool real;
        uint8_t base;
      } u_joint_p2p_insufficient_torque_for_planning;
      u_joint_p2p_insufficient_torque_for_planning.base = 0;
      u_joint_p2p_insufficient_torque_for_planning.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->joint_p2p_insufficient_torque_for_planning = u_joint_p2p_insufficient_torque_for_planning.real;
      offset += sizeof(this->joint_p2p_insufficient_torque_for_planning);
      union {
        bool real;
        uint8_t base;
      } u_tau_j_range_violation;
      u_tau_j_range_violation.base = 0;
      u_tau_j_range_violation.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->tau_j_range_violation = u_tau_j_range_violation.real;
      offset += sizeof(this->tau_j_range_violation);
      union {
        bool real;
        uint8_t base;
      } u_instability_detected;
      u_instability_detected.base = 0;
      u_instability_detected.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->instability_detected = u_instability_detected.real;
      offset += sizeof(this->instability_detected);
      union {
        bool real;
        uint8_t base;
      } u_joint_move_in_wrong_direction;
      u_joint_move_in_wrong_direction.base = 0;
      u_joint_move_in_wrong_direction.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->joint_move_in_wrong_direction = u_joint_move_in_wrong_direction.real;
      offset += sizeof(this->joint_move_in_wrong_direction);
      union {
        bool real;
        uint8_t base;
      } u_cartesian_spline_motion_generator_violation;
      u_cartesian_spline_motion_generator_violation.base = 0;
      u_cartesian_spline_motion_generator_violation.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->cartesian_spline_motion_generator_violation = u_cartesian_spline_motion_generator_violation.real;
      offset += sizeof(this->cartesian_spline_motion_generator_violation);
      union {
        bool real;
        uint8_t base;
      } u_joint_via_motion_generator_planning_joint_limit_violation;
      u_joint_via_motion_generator_planning_joint_limit_violation.base = 0;
      u_joint_via_motion_generator_planning_joint_limit_violation.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->joint_via_motion_generator_planning_joint_limit_violation = u_joint_via_motion_generator_planning_joint_limit_violation.real;
      offset += sizeof(this->joint_via_motion_generator_planning_joint_limit_violation);
      union {
        bool real;
        uint8_t base;
      } u_base_acceleration_initialization_timeout;
      u_base_acceleration_initialization_timeout.base = 0;
      u_base_acceleration_initialization_timeout.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->base_acceleration_initialization_timeout = u_base_acceleration_initialization_timeout.real;
      offset += sizeof(this->base_acceleration_initialization_timeout);
      union {
        bool real;
        uint8_t base;
      } u_base_acceleration_invalid_reading;
      u_base_acceleration_invalid_reading.base = 0;
      u_base_acceleration_invalid_reading.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->base_acceleration_invalid_reading = u_base_acceleration_invalid_reading.real;
      offset += sizeof(this->base_acceleration_invalid_reading);
     return offset;
    }

    virtual const char * getType() override { return "franka_msgs/Errors"; };
    virtual const char * getMD5() override { return "082e9a670c96d8bc64b53e32777458e7"; };

  };

}
#endif
