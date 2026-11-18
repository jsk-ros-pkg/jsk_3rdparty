#ifndef _ROS_nmea_msgs_Sentence_h
#define _ROS_nmea_msgs_Sentence_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"

namespace nmea_msgs
{

  class Sentence : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef const char* _sentence_type;
      _sentence_type sentence;

    Sentence():
      header(),
      sentence("")
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      uint32_t length_sentence = strlen(this->sentence);
      varToArr(outbuffer + offset, length_sentence);
      offset += 4;
      memcpy(outbuffer + offset, this->sentence, length_sentence);
      offset += length_sentence;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      uint32_t length_sentence;
      arrToVar(length_sentence, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_sentence; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_sentence-1]=0;
      this->sentence = (char *)(inbuffer + offset-1);
      offset += length_sentence;
     return offset;
    }

    virtual const char * getType() override { return "nmea_msgs/Sentence"; };
    virtual const char * getMD5() override { return "9f221efc5f4b3bac7ce4af102b32308b"; };

  };

}
#endif
