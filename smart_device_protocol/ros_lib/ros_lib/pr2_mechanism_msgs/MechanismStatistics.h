#ifndef _ROS_pr2_mechanism_msgs_MechanismStatistics_h
#define _ROS_pr2_mechanism_msgs_MechanismStatistics_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "pr2_mechanism_msgs/ActuatorStatistics.h"
#include "pr2_mechanism_msgs/JointStatistics.h"
#include "pr2_mechanism_msgs/ControllerStatistics.h"

namespace pr2_mechanism_msgs
{

  class MechanismStatistics : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      uint32_t actuator_statistics_length;
      typedef pr2_mechanism_msgs::ActuatorStatistics _actuator_statistics_type;
      _actuator_statistics_type st_actuator_statistics;
      _actuator_statistics_type * actuator_statistics;
      uint32_t joint_statistics_length;
      typedef pr2_mechanism_msgs::JointStatistics _joint_statistics_type;
      _joint_statistics_type st_joint_statistics;
      _joint_statistics_type * joint_statistics;
      uint32_t controller_statistics_length;
      typedef pr2_mechanism_msgs::ControllerStatistics _controller_statistics_type;
      _controller_statistics_type st_controller_statistics;
      _controller_statistics_type * controller_statistics;

    MechanismStatistics():
      header(),
      actuator_statistics_length(0), st_actuator_statistics(), actuator_statistics(nullptr),
      joint_statistics_length(0), st_joint_statistics(), joint_statistics(nullptr),
      controller_statistics_length(0), st_controller_statistics(), controller_statistics(nullptr)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      *(outbuffer + offset + 0) = (this->actuator_statistics_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->actuator_statistics_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->actuator_statistics_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->actuator_statistics_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->actuator_statistics_length);
      for( uint32_t i = 0; i < actuator_statistics_length; i++){
      offset += this->actuator_statistics[i].serialize(outbuffer + offset);
      }
      *(outbuffer + offset + 0) = (this->joint_statistics_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->joint_statistics_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->joint_statistics_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->joint_statistics_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->joint_statistics_length);
      for( uint32_t i = 0; i < joint_statistics_length; i++){
      offset += this->joint_statistics[i].serialize(outbuffer + offset);
      }
      *(outbuffer + offset + 0) = (this->controller_statistics_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->controller_statistics_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->controller_statistics_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->controller_statistics_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->controller_statistics_length);
      for( uint32_t i = 0; i < controller_statistics_length; i++){
      offset += this->controller_statistics[i].serialize(outbuffer + offset);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      uint32_t actuator_statistics_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      actuator_statistics_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      actuator_statistics_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      actuator_statistics_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->actuator_statistics_length);
      if(actuator_statistics_lengthT > actuator_statistics_length)
        this->actuator_statistics = (pr2_mechanism_msgs::ActuatorStatistics*)realloc(this->actuator_statistics, actuator_statistics_lengthT * sizeof(pr2_mechanism_msgs::ActuatorStatistics));
      actuator_statistics_length = actuator_statistics_lengthT;
      for( uint32_t i = 0; i < actuator_statistics_length; i++){
      offset += this->st_actuator_statistics.deserialize(inbuffer + offset);
        memcpy( &(this->actuator_statistics[i]), &(this->st_actuator_statistics), sizeof(pr2_mechanism_msgs::ActuatorStatistics));
      }
      uint32_t joint_statistics_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      joint_statistics_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      joint_statistics_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      joint_statistics_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->joint_statistics_length);
      if(joint_statistics_lengthT > joint_statistics_length)
        this->joint_statistics = (pr2_mechanism_msgs::JointStatistics*)realloc(this->joint_statistics, joint_statistics_lengthT * sizeof(pr2_mechanism_msgs::JointStatistics));
      joint_statistics_length = joint_statistics_lengthT;
      for( uint32_t i = 0; i < joint_statistics_length; i++){
      offset += this->st_joint_statistics.deserialize(inbuffer + offset);
        memcpy( &(this->joint_statistics[i]), &(this->st_joint_statistics), sizeof(pr2_mechanism_msgs::JointStatistics));
      }
      uint32_t controller_statistics_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      controller_statistics_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      controller_statistics_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      controller_statistics_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->controller_statistics_length);
      if(controller_statistics_lengthT > controller_statistics_length)
        this->controller_statistics = (pr2_mechanism_msgs::ControllerStatistics*)realloc(this->controller_statistics, controller_statistics_lengthT * sizeof(pr2_mechanism_msgs::ControllerStatistics));
      controller_statistics_length = controller_statistics_lengthT;
      for( uint32_t i = 0; i < controller_statistics_length; i++){
      offset += this->st_controller_statistics.deserialize(inbuffer + offset);
        memcpy( &(this->controller_statistics[i]), &(this->st_controller_statistics), sizeof(pr2_mechanism_msgs::ControllerStatistics));
      }
     return offset;
    }

    virtual const char * getType() override { return "pr2_mechanism_msgs/MechanismStatistics"; };
    virtual const char * getMD5() override { return "b4a99069393681672c01f8c823458e04"; };

  };

}
#endif
