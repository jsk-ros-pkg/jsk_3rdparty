#ifndef _ROS_SERVICE_MongoUpdateMsg_h
#define _ROS_SERVICE_MongoUpdateMsg_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "mongodb_store_msgs/StringPairList.h"
#include "mongodb_store_msgs/SerialisedMessage.h"

namespace mongodb_store_msgs
{

static const char MONGOUPDATEMSG[] = "mongodb_store_msgs/MongoUpdateMsg";

  class MongoUpdateMsgRequest : public ros::Msg
  {
    public:
      typedef const char* _database_type;
      _database_type database;
      typedef const char* _collection_type;
      _collection_type collection;
      typedef bool _upsert_type;
      _upsert_type upsert;
      typedef mongodb_store_msgs::StringPairList _message_query_type;
      _message_query_type message_query;
      typedef mongodb_store_msgs::StringPairList _meta_query_type;
      _meta_query_type meta_query;
      typedef mongodb_store_msgs::SerialisedMessage _message_type;
      _message_type message;
      typedef mongodb_store_msgs::StringPairList _meta_type;
      _meta_type meta;

    MongoUpdateMsgRequest():
      database(""),
      collection(""),
      upsert(0),
      message_query(),
      meta_query(),
      message(),
      meta()
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
      union {
        bool real;
        uint8_t base;
      } u_upsert;
      u_upsert.real = this->upsert;
      *(outbuffer + offset + 0) = (u_upsert.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->upsert);
      offset += this->message_query.serialize(outbuffer + offset);
      offset += this->meta_query.serialize(outbuffer + offset);
      offset += this->message.serialize(outbuffer + offset);
      offset += this->meta.serialize(outbuffer + offset);
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
      union {
        bool real;
        uint8_t base;
      } u_upsert;
      u_upsert.base = 0;
      u_upsert.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->upsert = u_upsert.real;
      offset += sizeof(this->upsert);
      offset += this->message_query.deserialize(inbuffer + offset);
      offset += this->meta_query.deserialize(inbuffer + offset);
      offset += this->message.deserialize(inbuffer + offset);
      offset += this->meta.deserialize(inbuffer + offset);
     return offset;
    }

    virtual const char * getType() override { return MONGOUPDATEMSG; };
    virtual const char * getMD5() override { return "5d87a90aa8c3d8f4cf31305f10951711"; };

  };

  class MongoUpdateMsgResponse : public ros::Msg
  {
    public:
      typedef const char* _id_type;
      _id_type id;
      typedef bool _success_type;
      _success_type success;

    MongoUpdateMsgResponse():
      id(""),
      success(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      uint32_t length_id = strlen(this->id);
      varToArr(outbuffer + offset, length_id);
      offset += 4;
      memcpy(outbuffer + offset, this->id, length_id);
      offset += length_id;
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
      uint32_t length_id;
      arrToVar(length_id, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_id; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_id-1]=0;
      this->id = (char *)(inbuffer + offset-1);
      offset += length_id;
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

    virtual const char * getType() override { return MONGOUPDATEMSG; };
    virtual const char * getMD5() override { return "eb98d6e8d810388b13fa8e5a365eec6a"; };

  };

  class MongoUpdateMsg {
    public:
    typedef MongoUpdateMsgRequest Request;
    typedef MongoUpdateMsgResponse Response;
  };

}
#endif
