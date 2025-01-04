#ifndef _ROS_face_detector_FaceDetectorResult_h
#define _ROS_face_detector_FaceDetectorResult_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "people_msgs/PositionMeasurement.h"

namespace face_detector
{

  class FaceDetectorResult : public ros::Msg
  {
    public:
      uint32_t face_positions_length;
      typedef people_msgs::PositionMeasurement _face_positions_type;
      _face_positions_type st_face_positions;
      _face_positions_type * face_positions;

    FaceDetectorResult():
      face_positions_length(0), st_face_positions(), face_positions(nullptr)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->face_positions_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->face_positions_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->face_positions_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->face_positions_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->face_positions_length);
      for( uint32_t i = 0; i < face_positions_length; i++){
      offset += this->face_positions[i].serialize(outbuffer + offset);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      uint32_t face_positions_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      face_positions_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      face_positions_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      face_positions_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->face_positions_length);
      if(face_positions_lengthT > face_positions_length)
        this->face_positions = (people_msgs::PositionMeasurement*)realloc(this->face_positions, face_positions_lengthT * sizeof(people_msgs::PositionMeasurement));
      face_positions_length = face_positions_lengthT;
      for( uint32_t i = 0; i < face_positions_length; i++){
      offset += this->st_face_positions.deserialize(inbuffer + offset);
        memcpy( &(this->face_positions[i]), &(this->st_face_positions), sizeof(people_msgs::PositionMeasurement));
      }
     return offset;
    }

    virtual const char * getType() override { return "face_detector/FaceDetectorResult"; };
    virtual const char * getMD5() override { return "b5dc843df183dbab7f0ab2f5ef5b6f9d"; };

  };

}
#endif
