#ifndef _ROS_google_chat_ros_Message_h
#define _ROS_google_chat_ros_Message_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "google_chat_ros/User.h"
#include "google_chat_ros/Annotation.h"
#include "google_chat_ros/Attachment.h"

namespace google_chat_ros
{

  class Message : public ros::Msg
  {
    public:
      typedef const char* _name_type;
      _name_type name;
      typedef google_chat_ros::User _sender_type;
      _sender_type sender;
      typedef const char* _create_time_type;
      _create_time_type create_time;
      typedef const char* _text_type;
      _text_type text;
      typedef const char* _thread_name_type;
      _thread_name_type thread_name;
      uint32_t annotations_length;
      typedef google_chat_ros::Annotation _annotations_type;
      _annotations_type st_annotations;
      _annotations_type * annotations;
      typedef const char* _argument_text_type;
      _argument_text_type argument_text;
      uint32_t attachments_length;
      typedef google_chat_ros::Attachment _attachments_type;
      _attachments_type st_attachments;
      _attachments_type * attachments;

    Message():
      name(""),
      sender(),
      create_time(""),
      text(""),
      thread_name(""),
      annotations_length(0), st_annotations(), annotations(nullptr),
      argument_text(""),
      attachments_length(0), st_attachments(), attachments(nullptr)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      uint32_t length_name = strlen(this->name);
      varToArr(outbuffer + offset, length_name);
      offset += 4;
      memcpy(outbuffer + offset, this->name, length_name);
      offset += length_name;
      offset += this->sender.serialize(outbuffer + offset);
      uint32_t length_create_time = strlen(this->create_time);
      varToArr(outbuffer + offset, length_create_time);
      offset += 4;
      memcpy(outbuffer + offset, this->create_time, length_create_time);
      offset += length_create_time;
      uint32_t length_text = strlen(this->text);
      varToArr(outbuffer + offset, length_text);
      offset += 4;
      memcpy(outbuffer + offset, this->text, length_text);
      offset += length_text;
      uint32_t length_thread_name = strlen(this->thread_name);
      varToArr(outbuffer + offset, length_thread_name);
      offset += 4;
      memcpy(outbuffer + offset, this->thread_name, length_thread_name);
      offset += length_thread_name;
      *(outbuffer + offset + 0) = (this->annotations_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->annotations_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->annotations_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->annotations_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->annotations_length);
      for( uint32_t i = 0; i < annotations_length; i++){
      offset += this->annotations[i].serialize(outbuffer + offset);
      }
      uint32_t length_argument_text = strlen(this->argument_text);
      varToArr(outbuffer + offset, length_argument_text);
      offset += 4;
      memcpy(outbuffer + offset, this->argument_text, length_argument_text);
      offset += length_argument_text;
      *(outbuffer + offset + 0) = (this->attachments_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->attachments_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->attachments_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->attachments_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->attachments_length);
      for( uint32_t i = 0; i < attachments_length; i++){
      offset += this->attachments[i].serialize(outbuffer + offset);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      uint32_t length_name;
      arrToVar(length_name, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_name; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_name-1]=0;
      this->name = (char *)(inbuffer + offset-1);
      offset += length_name;
      offset += this->sender.deserialize(inbuffer + offset);
      uint32_t length_create_time;
      arrToVar(length_create_time, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_create_time; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_create_time-1]=0;
      this->create_time = (char *)(inbuffer + offset-1);
      offset += length_create_time;
      uint32_t length_text;
      arrToVar(length_text, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_text; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_text-1]=0;
      this->text = (char *)(inbuffer + offset-1);
      offset += length_text;
      uint32_t length_thread_name;
      arrToVar(length_thread_name, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_thread_name; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_thread_name-1]=0;
      this->thread_name = (char *)(inbuffer + offset-1);
      offset += length_thread_name;
      uint32_t annotations_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      annotations_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      annotations_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      annotations_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->annotations_length);
      if(annotations_lengthT > annotations_length)
        this->annotations = (google_chat_ros::Annotation*)realloc(this->annotations, annotations_lengthT * sizeof(google_chat_ros::Annotation));
      annotations_length = annotations_lengthT;
      for( uint32_t i = 0; i < annotations_length; i++){
      offset += this->st_annotations.deserialize(inbuffer + offset);
        memcpy( &(this->annotations[i]), &(this->st_annotations), sizeof(google_chat_ros::Annotation));
      }
      uint32_t length_argument_text;
      arrToVar(length_argument_text, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_argument_text; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_argument_text-1]=0;
      this->argument_text = (char *)(inbuffer + offset-1);
      offset += length_argument_text;
      uint32_t attachments_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      attachments_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      attachments_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      attachments_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->attachments_length);
      if(attachments_lengthT > attachments_length)
        this->attachments = (google_chat_ros::Attachment*)realloc(this->attachments, attachments_lengthT * sizeof(google_chat_ros::Attachment));
      attachments_length = attachments_lengthT;
      for( uint32_t i = 0; i < attachments_length; i++){
      offset += this->st_attachments.deserialize(inbuffer + offset);
        memcpy( &(this->attachments[i]), &(this->st_attachments), sizeof(google_chat_ros::Attachment));
      }
     return offset;
    }

    virtual const char * getType() override { return "google_chat_ros/Message"; };
    virtual const char * getMD5() override { return "74482e9e8af0e668f3b70e9af9ccd33f"; };

  };

}
#endif
