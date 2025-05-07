#ifndef _ROS_robot_mechanism_controllers_JTCartesianControllerState_h
#define _ROS_robot_mechanism_controllers_JTCartesianControllerState_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Wrench.h"
#include "std_msgs/Float64MultiArray.h"

namespace robot_mechanism_controllers
{

  class JTCartesianControllerState : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef geometry_msgs::PoseStamped _x_type;
      _x_type x;
      typedef geometry_msgs::PoseStamped _x_desi_type;
      _x_desi_type x_desi;
      typedef geometry_msgs::PoseStamped _x_desi_filtered_type;
      _x_desi_filtered_type x_desi_filtered;
      typedef geometry_msgs::Twist _x_err_type;
      _x_err_type x_err;
      typedef geometry_msgs::Twist _xd_type;
      _xd_type xd;
      typedef geometry_msgs::Twist _xd_desi_type;
      _xd_desi_type xd_desi;
      typedef geometry_msgs::Wrench _F_type;
      _F_type F;
      uint32_t tau_pose_length;
      typedef float _tau_pose_type;
      _tau_pose_type st_tau_pose;
      _tau_pose_type * tau_pose;
      uint32_t tau_posture_length;
      typedef float _tau_posture_type;
      _tau_posture_type st_tau_posture;
      _tau_posture_type * tau_posture;
      uint32_t tau_length;
      typedef float _tau_type;
      _tau_type st_tau;
      _tau_type * tau;
      typedef std_msgs::Float64MultiArray _J_type;
      _J_type J;
      typedef std_msgs::Float64MultiArray _N_type;
      _N_type N;

    JTCartesianControllerState():
      header(),
      x(),
      x_desi(),
      x_desi_filtered(),
      x_err(),
      xd(),
      xd_desi(),
      F(),
      tau_pose_length(0), st_tau_pose(), tau_pose(nullptr),
      tau_posture_length(0), st_tau_posture(), tau_posture(nullptr),
      tau_length(0), st_tau(), tau(nullptr),
      J(),
      N()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      offset += this->x.serialize(outbuffer + offset);
      offset += this->x_desi.serialize(outbuffer + offset);
      offset += this->x_desi_filtered.serialize(outbuffer + offset);
      offset += this->x_err.serialize(outbuffer + offset);
      offset += this->xd.serialize(outbuffer + offset);
      offset += this->xd_desi.serialize(outbuffer + offset);
      offset += this->F.serialize(outbuffer + offset);
      *(outbuffer + offset + 0) = (this->tau_pose_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->tau_pose_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->tau_pose_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->tau_pose_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->tau_pose_length);
      for( uint32_t i = 0; i < tau_pose_length; i++){
      offset += serializeAvrFloat64(outbuffer + offset, this->tau_pose[i]);
      }
      *(outbuffer + offset + 0) = (this->tau_posture_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->tau_posture_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->tau_posture_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->tau_posture_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->tau_posture_length);
      for( uint32_t i = 0; i < tau_posture_length; i++){
      offset += serializeAvrFloat64(outbuffer + offset, this->tau_posture[i]);
      }
      *(outbuffer + offset + 0) = (this->tau_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->tau_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->tau_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->tau_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->tau_length);
      for( uint32_t i = 0; i < tau_length; i++){
      offset += serializeAvrFloat64(outbuffer + offset, this->tau[i]);
      }
      offset += this->J.serialize(outbuffer + offset);
      offset += this->N.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      offset += this->x.deserialize(inbuffer + offset);
      offset += this->x_desi.deserialize(inbuffer + offset);
      offset += this->x_desi_filtered.deserialize(inbuffer + offset);
      offset += this->x_err.deserialize(inbuffer + offset);
      offset += this->xd.deserialize(inbuffer + offset);
      offset += this->xd_desi.deserialize(inbuffer + offset);
      offset += this->F.deserialize(inbuffer + offset);
      uint32_t tau_pose_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      tau_pose_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      tau_pose_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      tau_pose_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->tau_pose_length);
      if(tau_pose_lengthT > tau_pose_length)
        this->tau_pose = (float*)realloc(this->tau_pose, tau_pose_lengthT * sizeof(float));
      tau_pose_length = tau_pose_lengthT;
      for( uint32_t i = 0; i < tau_pose_length; i++){
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->st_tau_pose));
        memcpy( &(this->tau_pose[i]), &(this->st_tau_pose), sizeof(float));
      }
      uint32_t tau_posture_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      tau_posture_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      tau_posture_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      tau_posture_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->tau_posture_length);
      if(tau_posture_lengthT > tau_posture_length)
        this->tau_posture = (float*)realloc(this->tau_posture, tau_posture_lengthT * sizeof(float));
      tau_posture_length = tau_posture_lengthT;
      for( uint32_t i = 0; i < tau_posture_length; i++){
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->st_tau_posture));
        memcpy( &(this->tau_posture[i]), &(this->st_tau_posture), sizeof(float));
      }
      uint32_t tau_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      tau_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      tau_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      tau_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->tau_length);
      if(tau_lengthT > tau_length)
        this->tau = (float*)realloc(this->tau, tau_lengthT * sizeof(float));
      tau_length = tau_lengthT;
      for( uint32_t i = 0; i < tau_length; i++){
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->st_tau));
        memcpy( &(this->tau[i]), &(this->st_tau), sizeof(float));
      }
      offset += this->J.deserialize(inbuffer + offset);
      offset += this->N.deserialize(inbuffer + offset);
     return offset;
    }

    virtual const char * getType() override { return "robot_mechanism_controllers/JTCartesianControllerState"; };
    virtual const char * getMD5() override { return "6ecdaa599ea0d27643819ae4cd4c43d0"; };

  };

}
#endif
