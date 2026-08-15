#ifndef _ROS_franka_example_controllers_JointTorqueComparison_h
#define _ROS_franka_example_controllers_JointTorqueComparison_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace franka_example_controllers
{

  class JointTorqueComparison : public ros::Msg
  {
    public:
      float tau_error[7];
      float tau_commanded[7];
      float tau_measured[7];
      typedef float _root_mean_square_error_type;
      _root_mean_square_error_type root_mean_square_error;

    JointTorqueComparison():
      tau_error(),
      tau_commanded(),
      tau_measured(),
      root_mean_square_error(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      for( uint32_t i = 0; i < 7; i++){
      offset += serializeAvrFloat64(outbuffer + offset, this->tau_error[i]);
      }
      for( uint32_t i = 0; i < 7; i++){
      offset += serializeAvrFloat64(outbuffer + offset, this->tau_commanded[i]);
      }
      for( uint32_t i = 0; i < 7; i++){
      offset += serializeAvrFloat64(outbuffer + offset, this->tau_measured[i]);
      }
      offset += serializeAvrFloat64(outbuffer + offset, this->root_mean_square_error);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      for( uint32_t i = 0; i < 7; i++){
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->tau_error[i]));
      }
      for( uint32_t i = 0; i < 7; i++){
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->tau_commanded[i]));
      }
      for( uint32_t i = 0; i < 7; i++){
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->tau_measured[i]));
      }
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->root_mean_square_error));
     return offset;
    }

    virtual const char * getType() override { return "franka_example_controllers/JointTorqueComparison"; };
    virtual const char * getMD5() override { return "6c09db90263c92a2e4e4d736f67bc033"; };

  };

}
#endif
