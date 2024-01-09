#ifndef _ROS_robot_calibration_msgs_CalibrationData_h
#define _ROS_robot_calibration_msgs_CalibrationData_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "sensor_msgs/JointState.h"
#include "robot_calibration_msgs/Observation.h"

namespace robot_calibration_msgs
{

  class CalibrationData : public ros::Msg
  {
    public:
      typedef sensor_msgs::JointState _joint_states_type;
      _joint_states_type joint_states;
      uint32_t observations_length;
      typedef robot_calibration_msgs::Observation _observations_type;
      _observations_type st_observations;
      _observations_type * observations;

    CalibrationData():
      joint_states(),
      observations_length(0), st_observations(), observations(nullptr)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->joint_states.serialize(outbuffer + offset);
      *(outbuffer + offset + 0) = (this->observations_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->observations_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->observations_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->observations_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->observations_length);
      for( uint32_t i = 0; i < observations_length; i++){
      offset += this->observations[i].serialize(outbuffer + offset);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += this->joint_states.deserialize(inbuffer + offset);
      uint32_t observations_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      observations_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      observations_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      observations_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->observations_length);
      if(observations_lengthT > observations_length)
        this->observations = (robot_calibration_msgs::Observation*)realloc(this->observations, observations_lengthT * sizeof(robot_calibration_msgs::Observation));
      observations_length = observations_lengthT;
      for( uint32_t i = 0; i < observations_length; i++){
      offset += this->st_observations.deserialize(inbuffer + offset);
        memcpy( &(this->observations[i]), &(this->st_observations), sizeof(robot_calibration_msgs::Observation));
      }
     return offset;
    }

    virtual const char * getType() override { return "robot_calibration_msgs/CalibrationData"; };
    virtual const char * getMD5() override { return "a9a1a8b53ea9e13de6dae25608004191"; };

  };

}
#endif
