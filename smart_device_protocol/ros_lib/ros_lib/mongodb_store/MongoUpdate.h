#ifndef _ROS_SERVICE_MongoUpdate_h
#define _ROS_SERVICE_MongoUpdate_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace mongodb_store
{

static const char MONGOUPDATE[] = "mongodb_store/MongoUpdate";

  class MongoUpdateRequest : public ros::Msg
  {
    public:
      typedef const char* _db_type;
      _db_type db;
      typedef const char* _collection_type;
      _collection_type collection;
      typedef const char* _query_type;
      _query_type query;
      typedef const char* _update_type;
      _update_type update;

    MongoUpdateRequest():
      db(""),
      collection(""),
      query(""),
      update("")
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
      uint32_t length_query = strlen(this->query);
      varToArr(outbuffer + offset, length_query);
      offset += 4;
      memcpy(outbuffer + offset, this->query, length_query);
      offset += length_query;
      uint32_t length_update = strlen(this->update);
      varToArr(outbuffer + offset, length_update);
      offset += 4;
      memcpy(outbuffer + offset, this->update, length_update);
      offset += length_update;
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
      uint32_t length_query;
      arrToVar(length_query, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_query; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_query-1]=0;
      this->query = (char *)(inbuffer + offset-1);
      offset += length_query;
      uint32_t length_update;
      arrToVar(length_update, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_update; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_update-1]=0;
      this->update = (char *)(inbuffer + offset-1);
      offset += length_update;
     return offset;
    }

    virtual const char * getType() override { return MONGOUPDATE; };
    virtual const char * getMD5() override { return "2b149bc6a0fea76ae96dfb48fcf24065"; };

  };

  class MongoUpdateResponse : public ros::Msg
  {
    public:
      typedef const char* _result_type;
      _result_type result;

    MongoUpdateResponse():
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

    virtual const char * getType() override { return MONGOUPDATE; };
    virtual const char * getMD5() override { return "c22f2a1ed8654a0b365f1bb3f7ff2c0f"; };

  };

  class MongoUpdate {
    public:
    typedef MongoUpdateRequest Request;
    typedef MongoUpdateResponse Response;
  };

}
#endif
