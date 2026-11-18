#ifndef _ROS_pr2_mechanism_controllers_OdometryMatrix_h
#define _ROS_pr2_mechanism_controllers_OdometryMatrix_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace pr2_mechanism_controllers
{

  class OdometryMatrix : public ros::Msg
  {
    public:
      uint32_t m_length;
      typedef float _m_type;
      _m_type st_m;
      _m_type * m;

    OdometryMatrix():
      m_length(0), st_m(), m(nullptr)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->m_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->m_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->m_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->m_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->m_length);
      for( uint32_t i = 0; i < m_length; i++){
      offset += serializeAvrFloat64(outbuffer + offset, this->m[i]);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      uint32_t m_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      m_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      m_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      m_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->m_length);
      if(m_lengthT > m_length)
        this->m = (float*)realloc(this->m, m_lengthT * sizeof(float));
      m_length = m_lengthT;
      for( uint32_t i = 0; i < m_length; i++){
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->st_m));
        memcpy( &(this->m[i]), &(this->st_m), sizeof(float));
      }
     return offset;
    }

    virtual const char * getType() override { return "pr2_mechanism_controllers/OdometryMatrix"; };
    virtual const char * getMD5() override { return "1f7818e7ce16454badf1bee936b0ff16"; };

  };

}
#endif
