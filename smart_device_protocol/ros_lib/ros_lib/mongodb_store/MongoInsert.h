#ifndef _ROS_SERVICE_MongoInsert_h
#define _ROS_SERVICE_MongoInsert_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace mongodb_store
{

static const char MONGOINSERT[] = "mongodb_store/MongoInsert";

  class MongoInsertRequest : public ros::Msg
  {
    public:
      typedef const char* _db_type;
      _db_type db;
      typedef const char* _collection_type;
      _collection_type collection;
      typedef const char* _document_type;
      _document_type document;

    MongoInsertRequest():
      db(""),
      collection(""),
      document("")
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      uint32_t length_db = strlen(this->db);
      varToArr(outbuffer + offset, length_db);
      offset += 4;
      memcpy(outbuffer + offset, this->db, length_db);
      offset += length_db;
      uint32_t length_collection = strlen(this->collection);
      varToArr(outbuffer + offset, length_collection);
      offset += 4;
      memcpy(outbuffer + offset, this->collection, length_collection);
      offset += length_collection;
      uint32_t length_document = strlen(this->document);
      varToArr(outbuffer + offset, length_document);
      offset += 4;
      memcpy(outbuffer + offset, this->document, length_document);
      offset += length_document;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      uint32_t length_db;
      arrToVar(length_db, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_db; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_db-1]=0;
      this->db = (char *)(inbuffer + offset-1);
      offset += length_db;
      uint32_t length_collection;
      arrToVar(length_collection, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_collection; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_collection-1]=0;
      this->collection = (char *)(inbuffer + offset-1);
      offset += length_collection;
      uint32_t length_document;
      arrToVar(length_document, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_document; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_document-1]=0;
      this->document = (char *)(inbuffer + offset-1);
      offset += length_document;
     return offset;
    }

    virtual const char * getType() override { return MONGOINSERT; };
    virtual const char * getMD5() override { return "370f65c72e031302f4aca6bcf64817f9"; };

  };

  class MongoInsertResponse : public ros::Msg
  {
    public:
      typedef const char* _result_type;
      _result_type result;

    MongoInsertResponse():
      result("")
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      uint32_t length_result = strlen(this->result);
      varToArr(outbuffer + offset, length_result);
      offset += 4;
      memcpy(outbuffer + offset, this->result, length_result);
      offset += length_result;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      uint32_t length_result;
      arrToVar(length_result, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_result; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_result-1]=0;
      this->result = (char *)(inbuffer + offset-1);
      offset += length_result;
     return offset;
    }

    virtual const char * getType() override { return MONGOINSERT; };
    virtual const char * getMD5() override { return "c22f2a1ed8654a0b365f1bb3f7ff2c0f"; };

  };

  class MongoInsert {
    public:
    typedef MongoInsertRequest Request;
    typedef MongoInsertResponse Response;
  };

}
#endif
