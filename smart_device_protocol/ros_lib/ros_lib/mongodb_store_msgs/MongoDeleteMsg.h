#ifndef _ROS_SERVICE_MongoDeleteMsg_h
#define _ROS_SERVICE_MongoDeleteMsg_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace mongodb_store_msgs
{

static const char MONGODELETEMSG[] = "mongodb_store_msgs/MongoDeleteMsg";

  class MongoDeleteMsgRequest : public ros::Msg
  {
    public:
      typedef const char* _database_type;
      _database_type database;
      typedef const char* _collection_type;
      _collection_type collection;
      typedef const char* _document_id_type;
      _document_id_type document_id;

    MongoDeleteMsgRequest():
      database(""),
      collection(""),
      document_id("")
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      uint32_t length_database = strlen(this->database);
      varToArr(outbuffer + offset, length_database);
      offset += 4;
      memcpy(outbuffer + offset, this->database, length_database);
      offset += length_database;
      uint32_t length_collection = strlen(this->collection);
      varToArr(outbuffer + offset, length_collection);
      offset += 4;
      memcpy(outbuffer + offset, this->collection, length_collection);
      offset += length_collection;
      uint32_t length_document_id = strlen(this->document_id);
      varToArr(outbuffer + offset, length_document_id);
      offset += 4;
      memcpy(outbuffer + offset, this->document_id, length_document_id);
      offset += length_document_id;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      uint32_t length_database;
      arrToVar(length_database, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_database; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_database-1]=0;
      this->database = (char *)(inbuffer + offset-1);
      offset += length_database;
      uint32_t length_collection;
      arrToVar(length_collection, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_collection; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_collection-1]=0;
      this->collection = (char *)(inbuffer + offset-1);
      offset += length_collection;
      uint32_t length_document_id;
      arrToVar(length_document_id, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_document_id; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_document_id-1]=0;
      this->document_id = (char *)(inbuffer + offset-1);
      offset += length_document_id;
     return offset;
    }

    virtual const char * getType() override { return MONGODELETEMSG; };
    virtual const char * getMD5() override { return "8db26da88c264ed1aced8ce3427e0db0"; };

  };

  class MongoDeleteMsgResponse : public ros::Msg
  {
    public:
      typedef bool _success_type;
      _success_type success;

    MongoDeleteMsgResponse():
      success(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_success;
      u_success.real = this->success;
      *(outbuffer + offset + 0) = (u_success.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->success);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_success;
      u_success.base = 0;
      u_success.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->success = u_success.real;
      offset += sizeof(this->success);
     return offset;
    }

    virtual const char * getType() override { return MONGODELETEMSG; };
    virtual const char * getMD5() override { return "358e233cde0c8a8bcfea4ce193f8fc15"; };

  };

  class MongoDeleteMsg {
    public:
    typedef MongoDeleteMsgRequest Request;
    typedef MongoDeleteMsgResponse Response;
  };

}
#endif
