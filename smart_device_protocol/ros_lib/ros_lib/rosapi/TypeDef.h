#ifndef _ROS_rosapi_TypeDef_h
#define _ROS_rosapi_TypeDef_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace rosapi
{

  class TypeDef : public ros::Msg
  {
    public:
      typedef const char* _type_type;
      _type_type type;
      uint32_t fieldnames_length;
      typedef char* _fieldnames_type;
      _fieldnames_type st_fieldnames;
      _fieldnames_type * fieldnames;
      uint32_t fieldtypes_length;
      typedef char* _fieldtypes_type;
      _fieldtypes_type st_fieldtypes;
      _fieldtypes_type * fieldtypes;
      uint32_t fieldarraylen_length;
      typedef int32_t _fieldarraylen_type;
      _fieldarraylen_type st_fieldarraylen;
      _fieldarraylen_type * fieldarraylen;
      uint32_t examples_length;
      typedef char* _examples_type;
      _examples_type st_examples;
      _examples_type * examples;
      uint32_t constnames_length;
      typedef char* _constnames_type;
      _constnames_type st_constnames;
      _constnames_type * constnames;
      uint32_t constvalues_length;
      typedef char* _constvalues_type;
      _constvalues_type st_constvalues;
      _constvalues_type * constvalues;

    TypeDef():
      type(""),
      fieldnames_length(0), st_fieldnames(), fieldnames(nullptr),
      fieldtypes_length(0), st_fieldtypes(), fieldtypes(nullptr),
      fieldarraylen_length(0), st_fieldarraylen(), fieldarraylen(nullptr),
      examples_length(0), st_examples(), examples(nullptr),
      constnames_length(0), st_constnames(), constnames(nullptr),
      constvalues_length(0), st_constvalues(), constvalues(nullptr)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      uint32_t length_type = strlen(this->type);
      varToArr(outbuffer + offset, length_type);
      offset += 4;
      memcpy(outbuffer + offset, this->type, length_type);
      offset += length_type;
      *(outbuffer + offset + 0) = (this->fieldnames_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->fieldnames_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->fieldnames_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->fieldnames_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->fieldnames_length);
      for( uint32_t i = 0; i < fieldnames_length; i++){
      uint32_t length_fieldnamesi = strlen(this->fieldnames[i]);
      varToArr(outbuffer + offset, length_fieldnamesi);
      offset += 4;
      memcpy(outbuffer + offset, this->fieldnames[i], length_fieldnamesi);
      offset += length_fieldnamesi;
      }
      *(outbuffer + offset + 0) = (this->fieldtypes_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->fieldtypes_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->fieldtypes_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->fieldtypes_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->fieldtypes_length);
      for( uint32_t i = 0; i < fieldtypes_length; i++){
      uint32_t length_fieldtypesi = strlen(this->fieldtypes[i]);
      varToArr(outbuffer + offset, length_fieldtypesi);
      offset += 4;
      memcpy(outbuffer + offset, this->fieldtypes[i], length_fieldtypesi);
      offset += length_fieldtypesi;
      }
      *(outbuffer + offset + 0) = (this->fieldarraylen_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->fieldarraylen_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->fieldarraylen_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->fieldarraylen_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->fieldarraylen_length);
      for( uint32_t i = 0; i < fieldarraylen_length; i++){
      union {
        int32_t real;
        uint32_t base;
      } u_fieldarrayleni;
      u_fieldarrayleni.real = this->fieldarraylen[i];
      *(outbuffer + offset + 0) = (u_fieldarrayleni.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_fieldarrayleni.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_fieldarrayleni.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_fieldarrayleni.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->fieldarraylen[i]);
      }
      *(outbuffer + offset + 0) = (this->examples_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->examples_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->examples_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->examples_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->examples_length);
      for( uint32_t i = 0; i < examples_length; i++){
      uint32_t length_examplesi = strlen(this->examples[i]);
      varToArr(outbuffer + offset, length_examplesi);
      offset += 4;
      memcpy(outbuffer + offset, this->examples[i], length_examplesi);
      offset += length_examplesi;
      }
      *(outbuffer + offset + 0) = (this->constnames_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->constnames_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->constnames_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->constnames_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->constnames_length);
      for( uint32_t i = 0; i < constnames_length; i++){
      uint32_t length_constnamesi = strlen(this->constnames[i]);
      varToArr(outbuffer + offset, length_constnamesi);
      offset += 4;
      memcpy(outbuffer + offset, this->constnames[i], length_constnamesi);
      offset += length_constnamesi;
      }
      *(outbuffer + offset + 0) = (this->constvalues_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->constvalues_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->constvalues_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->constvalues_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->constvalues_length);
      for( uint32_t i = 0; i < constvalues_length; i++){
      uint32_t length_constvaluesi = strlen(this->constvalues[i]);
      varToArr(outbuffer + offset, length_constvaluesi);
      offset += 4;
      memcpy(outbuffer + offset, this->constvalues[i], length_constvaluesi);
      offset += length_constvaluesi;
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      uint32_t length_type;
      arrToVar(length_type, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_type; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_type-1]=0;
      this->type = (char *)(inbuffer + offset-1);
      offset += length_type;
      uint32_t fieldnames_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      fieldnames_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      fieldnames_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      fieldnames_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->fieldnames_length);
      if(fieldnames_lengthT > fieldnames_length)
        this->fieldnames = (char**)realloc(this->fieldnames, fieldnames_lengthT * sizeof(char*));
      fieldnames_length = fieldnames_lengthT;
      for( uint32_t i = 0; i < fieldnames_length; i++){
      uint32_t length_st_fieldnames;
      arrToVar(length_st_fieldnames, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_st_fieldnames; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_st_fieldnames-1]=0;
      this->st_fieldnames = (char *)(inbuffer + offset-1);
      offset += length_st_fieldnames;
        memcpy( &(this->fieldnames[i]), &(this->st_fieldnames), sizeof(char*));
      }
      uint32_t fieldtypes_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      fieldtypes_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      fieldtypes_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      fieldtypes_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->fieldtypes_length);
      if(fieldtypes_lengthT > fieldtypes_length)
        this->fieldtypes = (char**)realloc(this->fieldtypes, fieldtypes_lengthT * sizeof(char*));
      fieldtypes_length = fieldtypes_lengthT;
      for( uint32_t i = 0; i < fieldtypes_length; i++){
      uint32_t length_st_fieldtypes;
      arrToVar(length_st_fieldtypes, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_st_fieldtypes; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_st_fieldtypes-1]=0;
      this->st_fieldtypes = (char *)(inbuffer + offset-1);
      offset += length_st_fieldtypes;
        memcpy( &(this->fieldtypes[i]), &(this->st_fieldtypes), sizeof(char*));
      }
      uint32_t fieldarraylen_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      fieldarraylen_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      fieldarraylen_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      fieldarraylen_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->fieldarraylen_length);
      if(fieldarraylen_lengthT > fieldarraylen_length)
        this->fieldarraylen = (int32_t*)realloc(this->fieldarraylen, fieldarraylen_lengthT * sizeof(int32_t));
      fieldarraylen_length = fieldarraylen_lengthT;
      for( uint32_t i = 0; i < fieldarraylen_length; i++){
      union {
        int32_t real;
        uint32_t base;
      } u_st_fieldarraylen;
      u_st_fieldarraylen.base = 0;
      u_st_fieldarraylen.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_fieldarraylen.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_st_fieldarraylen.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_st_fieldarraylen.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->st_fieldarraylen = u_st_fieldarraylen.real;
      offset += sizeof(this->st_fieldarraylen);
        memcpy( &(this->fieldarraylen[i]), &(this->st_fieldarraylen), sizeof(int32_t));
      }
      uint32_t examples_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      examples_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      examples_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      examples_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->examples_length);
      if(examples_lengthT > examples_length)
        this->examples = (char**)realloc(this->examples, examples_lengthT * sizeof(char*));
      examples_length = examples_lengthT;
      for( uint32_t i = 0; i < examples_length; i++){
      uint32_t length_st_examples;
      arrToVar(length_st_examples, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_st_examples; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_st_examples-1]=0;
      this->st_examples = (char *)(inbuffer + offset-1);
      offset += length_st_examples;
        memcpy( &(this->examples[i]), &(this->st_examples), sizeof(char*));
      }
      uint32_t constnames_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      constnames_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      constnames_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      constnames_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->constnames_length);
      if(constnames_lengthT > constnames_length)
        this->constnames = (char**)realloc(this->constnames, constnames_lengthT * sizeof(char*));
      constnames_length = constnames_lengthT;
      for( uint32_t i = 0; i < constnames_length; i++){
      uint32_t length_st_constnames;
      arrToVar(length_st_constnames, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_st_constnames; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_st_constnames-1]=0;
      this->st_constnames = (char *)(inbuffer + offset-1);
      offset += length_st_constnames;
        memcpy( &(this->constnames[i]), &(this->st_constnames), sizeof(char*));
      }
      uint32_t constvalues_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      constvalues_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      constvalues_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      constvalues_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->constvalues_length);
      if(constvalues_lengthT > constvalues_length)
        this->constvalues = (char**)realloc(this->constvalues, constvalues_lengthT * sizeof(char*));
      constvalues_length = constvalues_lengthT;
      for( uint32_t i = 0; i < constvalues_length; i++){
      uint32_t length_st_constvalues;
      arrToVar(length_st_constvalues, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_st_constvalues; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_st_constvalues-1]=0;
      this->st_constvalues = (char *)(inbuffer + offset-1);
      offset += length_st_constvalues;
        memcpy( &(this->constvalues[i]), &(this->st_constvalues), sizeof(char*));
      }
     return offset;
    }

    virtual const char * getType() override { return "rosapi/TypeDef"; };
    virtual const char * getMD5() override { return "80597571d79bbeef6c9c4d98f30116a0"; };

  };

}
#endif
