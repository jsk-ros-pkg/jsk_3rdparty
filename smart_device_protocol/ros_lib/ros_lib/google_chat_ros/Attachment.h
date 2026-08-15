#ifndef _ROS_google_chat_ros_Attachment_h
#define _ROS_google_chat_ros_Attachment_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace google_chat_ros
{

  class Attachment : public ros::Msg
  {
    public:
      typedef const char* _name_type;
      _name_type name;
      typedef const char* _content_name_type;
      _content_name_type content_name;
      typedef const char* _content_type_type;
      _content_type_type content_type;
      typedef const char* _thumnail_uri_type;
      _thumnail_uri_type thumnail_uri;
      typedef const char* _download_uri_type;
      _download_uri_type download_uri;
      typedef const char* _localpath_type;
      _localpath_type localpath;
      typedef bool _drive_file_type;
      _drive_file_type drive_file;
      typedef bool _uploaded_content_type;
      _uploaded_content_type uploaded_content;
      typedef const char* _attachment_resource_name_type;
      _attachment_resource_name_type attachment_resource_name;
      typedef const char* _drive_field_id_type;
      _drive_field_id_type drive_field_id;

    Attachment():
      name(""),
      content_name(""),
      content_type(""),
      thumnail_uri(""),
      download_uri(""),
      localpath(""),
      drive_file(0),
      uploaded_content(0),
      attachment_resource_name(""),
      drive_field_id("")
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
      uint32_t length_content_name = strlen(this->content_name);
      varToArr(outbuffer + offset, length_content_name);
      offset += 4;
      memcpy(outbuffer + offset, this->content_name, length_content_name);
      offset += length_content_name;
      uint32_t length_content_type = strlen(this->content_type);
      varToArr(outbuffer + offset, length_content_type);
      offset += 4;
      memcpy(outbuffer + offset, this->content_type, length_content_type);
      offset += length_content_type;
      uint32_t length_thumnail_uri = strlen(this->thumnail_uri);
      varToArr(outbuffer + offset, length_thumnail_uri);
      offset += 4;
      memcpy(outbuffer + offset, this->thumnail_uri, length_thumnail_uri);
      offset += length_thumnail_uri;
      uint32_t length_download_uri = strlen(this->download_uri);
      varToArr(outbuffer + offset, length_download_uri);
      offset += 4;
      memcpy(outbuffer + offset, this->download_uri, length_download_uri);
      offset += length_download_uri;
      uint32_t length_localpath = strlen(this->localpath);
      varToArr(outbuffer + offset, length_localpath);
      offset += 4;
      memcpy(outbuffer + offset, this->localpath, length_localpath);
      offset += length_localpath;
      union {
        bool real;
        uint8_t base;
      } u_drive_file;
      u_drive_file.real = this->drive_file;
      *(outbuffer + offset + 0) = (u_drive_file.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->drive_file);
      union {
        bool real;
        uint8_t base;
      } u_uploaded_content;
      u_uploaded_content.real = this->uploaded_content;
      *(outbuffer + offset + 0) = (u_uploaded_content.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->uploaded_content);
      uint32_t length_attachment_resource_name = strlen(this->attachment_resource_name);
      varToArr(outbuffer + offset, length_attachment_resource_name);
      offset += 4;
      memcpy(outbuffer + offset, this->attachment_resource_name, length_attachment_resource_name);
      offset += length_attachment_resource_name;
      uint32_t length_drive_field_id = strlen(this->drive_field_id);
      varToArr(outbuffer + offset, length_drive_field_id);
      offset += 4;
      memcpy(outbuffer + offset, this->drive_field_id, length_drive_field_id);
      offset += length_drive_field_id;
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
      uint32_t length_content_name;
      arrToVar(length_content_name, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_content_name; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_content_name-1]=0;
      this->content_name = (char *)(inbuffer + offset-1);
      offset += length_content_name;
      uint32_t length_content_type;
      arrToVar(length_content_type, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_content_type; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_content_type-1]=0;
      this->content_type = (char *)(inbuffer + offset-1);
      offset += length_content_type;
      uint32_t length_thumnail_uri;
      arrToVar(length_thumnail_uri, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_thumnail_uri; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_thumnail_uri-1]=0;
      this->thumnail_uri = (char *)(inbuffer + offset-1);
      offset += length_thumnail_uri;
      uint32_t length_download_uri;
      arrToVar(length_download_uri, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_download_uri; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_download_uri-1]=0;
      this->download_uri = (char *)(inbuffer + offset-1);
      offset += length_download_uri;
      uint32_t length_localpath;
      arrToVar(length_localpath, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_localpath; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_localpath-1]=0;
      this->localpath = (char *)(inbuffer + offset-1);
      offset += length_localpath;
      union {
        bool real;
        uint8_t base;
      } u_drive_file;
      u_drive_file.base = 0;
      u_drive_file.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->drive_file = u_drive_file.real;
      offset += sizeof(this->drive_file);
      union {
        bool real;
        uint8_t base;
      } u_uploaded_content;
      u_uploaded_content.base = 0;
      u_uploaded_content.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->uploaded_content = u_uploaded_content.real;
      offset += sizeof(this->uploaded_content);
      uint32_t length_attachment_resource_name;
      arrToVar(length_attachment_resource_name, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_attachment_resource_name; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_attachment_resource_name-1]=0;
      this->attachment_resource_name = (char *)(inbuffer + offset-1);
      offset += length_attachment_resource_name;
      uint32_t length_drive_field_id;
      arrToVar(length_drive_field_id, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_drive_field_id; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_drive_field_id-1]=0;
      this->drive_field_id = (char *)(inbuffer + offset-1);
      offset += length_drive_field_id;
     return offset;
    }

    virtual const char * getType() override { return "google_chat_ros/Attachment"; };
    virtual const char * getMD5() override { return "4c5e6ebea127240726d3a3e9429557a1"; };

  };

}
#endif
