#ifndef _ROS_naoqi_bridge_msgs_SetSpeechVocabularyGoal_h
#define _ROS_naoqi_bridge_msgs_SetSpeechVocabularyGoal_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace naoqi_bridge_msgs
{

  class SetSpeechVocabularyGoal : public ros::Msg
  {
    public:
      uint32_t words_length;
      typedef char* _words_type;
      _words_type st_words;
      _words_type * words;

    SetSpeechVocabularyGoal():
      words_length(0), st_words(), words(nullptr)
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
     return offset;
    }

    virtual const char * getType() override { return "naoqi_bridge_msgs/SetSpeechVocabularyGoal"; };
    virtual const char * getMD5() override { return "2bd0e7dd008cf8f52a5113ba090403b7"; };

  };

}
#endif
