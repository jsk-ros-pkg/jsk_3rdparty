#ifndef _ROS_SERVICE_MongoQueryMsg_h
#define _ROS_SERVICE_MongoQueryMsg_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "mongodb_store_msgs/StringPairList.h"
#include "mongodb_store_msgs/SerialisedMessage.h"

namespace mongodb_store_msgs
{

static const char MONGOQUERYMSG[] = "mongodb_store_msgs/MongoQueryMsg";

  class MongoQueryMsgRequest : public ros::Msg
  {
    public:
      typedef const char* _database_type;
      _database_type database;
      typedef const char* _collection_type;
      _collection_type collection;
      typedef const char* _type_type;
      _type_type type;
      typedef bool _single_type;
      _single_type single;
      typedef uint16_t _limit_type;
      _limit_type limit;
      typedef mongodb_store_msgs::StringPairList _message_query_type;
      _message_query_type message_query;
      typedef mongodb_store_msgs::StringPairList _meta_query_type;
      _meta_query_type meta_query;
      typedef mongodb_store_msgs::StringPairList _sort_query_type;
      _sort_query_type sort_query;
      typedef mongodb_store_msgs::StringPairList _projection_query_type;
      _projection_query_type projection_query;
      enum { JSON_QUERY = "jnsdfskajd_fmgs.dlf" };

    MongoQueryMsgRequest():
      database(""),
      collection(""),
      type(""),
      single(0),
      limit(0),
      message_query(),
      meta_query(),
      sort_query(),
      projection_query()
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
      uint32_t length_type = strlen(this->type);
      varToArr(outbuffer + offset, length_type);
      offset += 4;
      memcpy(outbuffer + offset, this->type, length_type);
      offset += length_type;
      union {
        bool real;
        uint8_t base;
      } u_single;
      u_single.real = this->single;
      *(outbuffer + offset + 0) = (u_single.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->single);
      *(outbuffer + offset + 0) = (this->limit >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->limit >> (8 * 1)) & 0xFF;
      offset += sizeof(this->limit);
      offset += this->message_query.serialize(outbuffer + offset);
      offset += this->meta_query.serialize(outbuffer + offset);
      offset += this->sort_query.serialize(outbuffer + offset);
      offset += this->projection_query.serialize(outbuffer + offset);
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
      uint32_t length_type;
      arrToVar(length_type, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_type; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_type-1]=0;
      this->type = (char *)(inbuffer + offset-1);
      offset += length_type;
      union {
        bool real;
        uint8_t base;
      } u_single;
      u_single.base = 0;
      u_single.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->single = u_single.real;
      offset += sizeof(this->single);
      this->limit =  ((uint16_t) (*(inbuffer + offset)));
      this->limit |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->limit);
      offset += this->message_query.deserialize(inbuffer + offset);
      offset += this->meta_query.deserialize(inbuffer + offset);
      offset += this->sort_query.deserialize(inbuffer + offset);
      offset += this->projection_query.deserialize(inbuffer + offset);
     return offset;
    }

    virtual const char * getType() override { return MONGOQUERYMSG; };
    virtual const char * getMD5() override { return "3dce95387658eb89ce25e603efe525cd"; };

  };

  class MongoQueryMsgResponse : public ros::Msg
  {
    public:
      uint32_t messages_length;
      typedef mongodb_store_msgs::SerialisedMessage _messages_type;
      _messages_type st_messages;
      _messages_type * messages;
      uint32_t metas_length;
      typedef mongodb_store_msgs::StringPairList _metas_type;
      _metas_type st_metas;
      _metas_type * metas;

    MongoQueryMsgResponse():
      messages_length(0), st_messages(), messages(nullptr),
      metas_length(0), st_metas(), metas(nullptr)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->messages_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->messages_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->messages_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->messages_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->messages_length);
      for( uint32_t i = 0; i < messages_length; i++){
      offset += this->messages[i].serialize(outbuffer + offset);
      }
      *(outbuffer + offset + 0) = (this->metas_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->metas_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->metas_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->metas_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->metas_length);
      for( uint32_t i = 0; i < metas_length; i++){
      offset += this->metas[i].serialize(outbuffer + offset);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      uint32_t messages_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      messages_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      messages_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      messages_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->messages_length);
      if(messages_lengthT > messages_length)
        this->messages = (mongodb_store_msgs::SerialisedMessage*)realloc(this->messages, messages_lengthT * sizeof(mongodb_store_msgs::SerialisedMessage));
      messages_length = messages_lengthT;
      for( uint32_t i = 0; i < messages_length; i++){
      offset += this->st_messages.deserialize(inbuffer + offset);
        memcpy( &(this->messages[i]), &(this->st_messages), sizeof(mongodb_store_msgs::SerialisedMessage));
      }
      uint32_t metas_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      metas_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      metas_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      metas_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->metas_length);
      if(metas_lengthT > metas_length)
        this->metas = (mongodb_store_msgs::StringPairList*)realloc(this->metas, metas_lengthT * sizeof(mongodb_store_msgs::StringPairList));
      metas_length = metas_lengthT;
      for( uint32_t i = 0; i < metas_length; i++){
      offset += this->st_metas.deserialize(inbuffer + offset);
        memcpy( &(this->metas[i]), &(this->st_metas), sizeof(mongodb_store_msgs::StringPairList));
      }
     return offset;
    }

    virtual const char * getType() override { return MONGOQUERYMSG; };
    virtual const char * getMD5() override { return "f348d453c2d7347807f66360b61cd0ef"; };

  };

  class MongoQueryMsg {
    public:
    typedef MongoQueryMsgRequest Request;
    typedef MongoQueryMsgResponse Response;
  };

}
#endif
