#ifndef _ROS_jsk_recognition_msgs_ModelCoefficientsArray_h
#define _ROS_jsk_recognition_msgs_ModelCoefficientsArray_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "pcl_msgs/ModelCoefficients.h"

namespace jsk_recognition_msgs
{

  class ModelCoefficientsArray : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      uint32_t coefficients_length;
      typedef pcl_msgs::ModelCoefficients _coefficients_type;
      _coefficients_type st_coefficients;
      _coefficients_type * coefficients;

    ModelCoefficientsArray():
      header(),
      coefficients_length(0), st_coefficients(), coefficients(nullptr)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      *(outbuffer + offset + 0) = (this->coefficients_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->coefficients_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->coefficients_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->coefficients_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->coefficients_length);
      for( uint32_t i = 0; i < coefficients_length; i++){
      offset += this->coefficients[i].serialize(outbuffer + offset);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      uint32_t coefficients_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      coefficients_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      coefficients_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      coefficients_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->coefficients_length);
      if(coefficients_lengthT > coefficients_length)
        this->coefficients = (pcl_msgs::ModelCoefficients*)realloc(this->coefficients, coefficients_lengthT * sizeof(pcl_msgs::ModelCoefficients));
      coefficients_length = coefficients_lengthT;
      for( uint32_t i = 0; i < coefficients_length; i++){
      offset += this->st_coefficients.deserialize(inbuffer + offset);
        memcpy( &(this->coefficients[i]), &(this->st_coefficients), sizeof(pcl_msgs::ModelCoefficients));
      }
     return offset;
    }

    virtual const char * getType() override { return "jsk_recognition_msgs/ModelCoefficientsArray"; };
    virtual const char * getMD5() override { return "059efee897c3f4ae027a493e30c4c26b"; };

  };

}
#endif
