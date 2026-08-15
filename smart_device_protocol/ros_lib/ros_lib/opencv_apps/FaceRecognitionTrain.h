#ifndef _ROS_SERVICE_FaceRecognitionTrain_h
#define _ROS_SERVICE_FaceRecognitionTrain_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "opencv_apps/Rect.h"
#include "sensor_msgs/Image.h"

namespace opencv_apps
{

static const char FACERECOGNITIONTRAIN[] = "opencv_apps/FaceRecognitionTrain";

  class FaceRecognitionTrainRequest : public ros::Msg
  {
    public:
      uint32_t images_length;
      typedef sensor_msgs::Image _images_type;
      _images_type st_images;
      _images_type * images;
      uint32_t rects_length;
      typedef opencv_apps::Rect _rects_type;
      _rects_type st_rects;
      _rects_type * rects;
      uint32_t labels_length;
      typedef char* _labels_type;
      _labels_type st_labels;
      _labels_type * labels;

    FaceRecognitionTrainRequest():
      images_length(0), st_images(), images(nullptr),
      rects_length(0), st_rects(), rects(nullptr),
      labels_length(0), st_labels(), labels(nullptr)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->images_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->images_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->images_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->images_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->images_length);
      for( uint32_t i = 0; i < images_length; i++){
      offset += this->images[i].serialize(outbuffer + offset);
      }
      *(outbuffer + offset + 0) = (this->rects_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->rects_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->rects_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->rects_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->rects_length);
      for( uint32_t i = 0; i < rects_length; i++){
      offset += this->rects[i].serialize(outbuffer + offset);
      }
      *(outbuffer + offset + 0) = (this->labels_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->labels_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->labels_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->labels_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->labels_length);
      for( uint32_t i = 0; i < labels_length; i++){
      uint32_t length_labelsi = strlen(this->labels[i]);
      varToArr(outbuffer + offset, length_labelsi);
      offset += 4;
      memcpy(outbuffer + offset, this->labels[i], length_labelsi);
      offset += length_labelsi;
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      uint32_t images_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      images_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      images_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      images_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->images_length);
      if(images_lengthT > images_length)
        this->images = (sensor_msgs::Image*)realloc(this->images, images_lengthT * sizeof(sensor_msgs::Image));
      images_length = images_lengthT;
      for( uint32_t i = 0; i < images_length; i++){
      offset += this->st_images.deserialize(inbuffer + offset);
        memcpy( &(this->images[i]), &(this->st_images), sizeof(sensor_msgs::Image));
      }
      uint32_t rects_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      rects_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      rects_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      rects_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->rects_length);
      if(rects_lengthT > rects_length)
        this->rects = (opencv_apps::Rect*)realloc(this->rects, rects_lengthT * sizeof(opencv_apps::Rect));
      rects_length = rects_lengthT;
      for( uint32_t i = 0; i < rects_length; i++){
      offset += this->st_rects.deserialize(inbuffer + offset);
        memcpy( &(this->rects[i]), &(this->st_rects), sizeof(opencv_apps::Rect));
      }
      uint32_t labels_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      labels_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      labels_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      labels_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->labels_length);
      if(labels_lengthT > labels_length)
        this->labels = (char**)realloc(this->labels, labels_lengthT * sizeof(char*));
      labels_length = labels_lengthT;
      for( uint32_t i = 0; i < labels_length; i++){
      uint32_t length_st_labels;
      arrToVar(length_st_labels, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_st_labels; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_st_labels-1]=0;
      this->st_labels = (char *)(inbuffer + offset-1);
      offset += length_st_labels;
        memcpy( &(this->labels[i]), &(this->st_labels), sizeof(char*));
      }
     return offset;
    }

    virtual const char * getType() override { return FACERECOGNITIONTRAIN; };
    virtual const char * getMD5() override { return "ba188b4bf792edbaf69c7f296a16e0ec"; };

  };

  class FaceRecognitionTrainResponse : public ros::Msg
  {
    public:
      typedef bool _ok_type;
      _ok_type ok;
      typedef const char* _error_type;
      _error_type error;

    FaceRecognitionTrainResponse():
      ok(0),
      error("")
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_ok;
      u_ok.real = this->ok;
      *(outbuffer + offset + 0) = (u_ok.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->ok);
      uint32_t length_error = strlen(this->error);
      varToArr(outbuffer + offset, length_error);
      offset += 4;
      memcpy(outbuffer + offset, this->error, length_error);
      offset += length_error;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_ok;
      u_ok.base = 0;
      u_ok.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->ok = u_ok.real;
      offset += sizeof(this->ok);
      uint32_t length_error;
      arrToVar(length_error, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_error; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_error-1]=0;
      this->error = (char *)(inbuffer + offset-1);
      offset += length_error;
     return offset;
    }

    virtual const char * getType() override { return FACERECOGNITIONTRAIN; };
    virtual const char * getMD5() override { return "14d6fca830116fb9833d983a296f00ed"; };

  };

  class FaceRecognitionTrain {
    public:
    typedef FaceRecognitionTrainRequest Request;
    typedef FaceRecognitionTrainResponse Response;
  };

}
#endif
