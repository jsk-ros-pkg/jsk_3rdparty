#ifndef _ROS_naoqi_bridge_msgs_WordRecognized_h
#define _ROS_naoqi_bridge_msgs_WordRecognized_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace naoqi_bridge_msgs
{

  class WordRecognized : public ros::Msg
  {
    public:
      uint32_t words_length;
      typedef char* _words_type;
      _words_type st_words;
      _words_type * words;
      uint32_t confidence_values_length;
      typedef float _confidence_values_type;
      _confidence_values_type st_confidence_values;
      _confidence_values_type * confidence_values;

    WordRecognized():
      words_length(0), st_words(), words(nullptr),
      confidence_values_length(0), st_confidence_values(), confidence_values(nullptr)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->words_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->words_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->words_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->words_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->words_length);
      for( uint32_t i = 0; i < words_length; i++){
      uint32_t length_wordsi = strlen(this->words[i]);
      varToArr(outbuffer + offset, length_wordsi);
      offset += 4;
      memcpy(outbuffer + offset, this->words[i], length_wordsi);
      offset += length_wordsi;
      }
      *(outbuffer + offset + 0) = (this->confidence_values_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->confidence_values_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->confidence_values_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->confidence_values_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->confidence_values_length);
      for( uint32_t i = 0; i < confidence_values_length; i++){
      union {
        float real;
        uint32_t base;
      } u_confidence_valuesi;
      u_confidence_valuesi.real = this->confidence_values[i];
      *(outbuffer + offset + 0) = (u_confidence_valuesi.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_confidence_valuesi.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_confidence_valuesi.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_confidence_valuesi.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->confidence_values[i]);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      uint32_t words_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      words_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      words_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      words_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->words_length);
      if(words_lengthT > words_length)
        this->words = (char**)realloc(this->words, words_lengthT * sizeof(char*));
      words_length = words_lengthT;
      for( uint32_t i = 0; i < words_length; i++){
      uint32_t length_st_words;
      arrToVar(length_st_words, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_st_words; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_st_words-1]=0;
      this->st_words = (char *)(inbuffer + offset-1);
      offset += length_st_words;
        memcpy( &(this->words[i]), &(this->st_words), sizeof(char*));
      }
      uint32_t confidence_values_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      confidence_values_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      confidence_values_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      confidence_values_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->confidence_values_length);
      if(confidence_values_lengthT > confidence_values_length)
        this->confidence_values = (float*)realloc(this->confidence_values, confidence_values_lengthT * sizeof(float));
      confidence_values_length = confidence_values_lengthT;
      for( uint32_t i = 0; i < confidence_values_length; i++){
      union {
        float real;
        uint32_t base;
      } u_st_confidence_values;
      u_st_confidence_values.base = 0;
      u_st_confidence_values.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_confidence_values.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_st_confidence_values.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_st_confidence_values.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->st_confidence_values = u_st_confidence_values.real;
      offset += sizeof(this->st_confidence_values);
        memcpy( &(this->confidence_values[i]), &(this->st_confidence_values), sizeof(float));
      }
     return offset;
    }

    virtual const char * getType() override { return "naoqi_bridge_msgs/WordRecognized"; };
    virtual const char * getMD5() override { return "29134437cd61021f75f35f21b72b7eab"; };

  };

}
#endif
