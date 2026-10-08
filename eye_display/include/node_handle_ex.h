#pragma once

#include "ros/node_handle.h"
#include <vector>

namespace ros
{
template<class Hardware,
         int MAX_SUBSCRIBERS = 25,
         int MAX_PUBLISHERS = 25,
         int INPUT_SIZE = 512,
         int OUTPUT_SIZE = 512>
class NodeHandleEx : public NodeHandle_<Hardware, MAX_SUBSCRIBERS, MAX_PUBLISHERS, INPUT_SIZE, OUTPUT_SIZE>
{
  typedef NodeHandle_<Hardware, MAX_SUBSCRIBERS, MAX_PUBLISHERS, INPUT_SIZE, OUTPUT_SIZE> super;
public:
  bool getParam(const char* name, int* param, int length = 1, int timeout = 1000) {
    return super::getParam(name, param, length, timeout);
  }
  bool getParam(const char* name, float* param, int length = 1, int timeout = 1000) {
    logwarn("Failed to get param: length mismatch float");
    return super::getParam(name, param, length, timeout);
  }
  bool getParam(const char* name, char** param, int length = 1, int timeout = 1000) {
    return super::getParam(name, param, length, timeout);
  }
  bool getParam(const char* name, bool* param, int length = 1, int timeout = 1000) {
    return super::getParam(name, param, length, timeout);
  }
  bool getParam(const char* name, std::vector<std::string> &param, int timeout = 1000) {
    if (!super::requestParam(name, timeout)) return false;
    param.resize(super::req_param_resp.strings_length);
    //copy it over
    for (int i = 0; i < super::req_param_resp.strings_length; i++)
      param[i] = std::string(super::req_param_resp.strings[i]);
    return true;
  }
  bool getParam(const char* name, std::string &param, int timeout = 1000) {
    if (!super::requestParam(name, timeout)) return false;
    if (super::req_param_resp.strings_length == 1) {
      param = std::string(super::req_param_resp.strings[0]);
      return true;
    } else {
      logwarn("Failed to get param: length mismatch std::string");
      logwarn("Failed to get param: length mismatch");
      return false;
    }
  }
  bool getParam(const char *name, std::vector<int> &param, int timeout = 1000)
  {
      if (!super::requestParam(name, timeout))
          return false;
      param.resize(super::req_param_resp.ints_length);
      // copy it over
      for (int i = 0; i < super::req_param_resp.ints_length; i++)
          param[i] = super::req_param_resp.ints[i];
      return true;
  }

#define def_char_log_formatter(funcname)                        \
  void funcname(const char *format, ...) {                      \
    char *string;                                               \
    va_list args;                                               \
    va_start(args, format);                                     \
    if (0 > vasprintf(&string, format, args)) string = NULL;    \
    va_end(args);                                               \
    if(string) {                                                \
      super::funcname(string);                                  \
      free(string);                                             \
    }                                                           \
  }
  def_char_log_formatter(logdebug)
  def_char_log_formatter(loginfo)
  def_char_log_formatter(logwarn)
  def_char_log_formatter(logerror)
  def_char_log_formatter(logfatal)
};
}
