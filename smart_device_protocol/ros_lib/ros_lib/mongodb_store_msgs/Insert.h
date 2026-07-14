#ifndef _ROS_mongodb_store_msgs_Insert_h
#define _ROS_mongodb_store_msgs_Insert_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "mongodb_store_msgs/SerialisedMessage.h"
#include "mongodb_store_msgs/StringPairList.h"

namespace mongodb_store_msgs
{

  class Insert : public ros::Msg
  {
    public:
      typedef const char* _database_type;
      _database_type database;
      typedef const char* _collection_type;
      _collection_type collection;
      typedef mongodb_store_msgs::SerialisedMessage _message_type;
      _message_type message;
      typedef mongodb_store_msgs::StringPairList _meta_type;
      _meta_type meta;

    Insert():
      database(""),
      collection(""),
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
      offset += this->message.deserialize(inbuffer + offset);
      offset += this->meta.deserialize(inbuffer + offset);
     return offset;
    }

    virtual const char * getType() override { return "mongodb_store_msgs/Insert"; };
    virtual const char * getMD5() override { return "d071b179071167c692331b5356e30470"; };

  };

}
#endif
