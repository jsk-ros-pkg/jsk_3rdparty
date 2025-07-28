#pragma once

#include <ArduinoJson.h>
#include <iterator>
#include <sstream>

#include "node_handle_ex.h"  // #include "ros/node_handle.h"
#include "geometry_msgs/Point.h"
#include "std_msgs/String.h"

void callback_look_at(const geometry_msgs::Point &msg);
void callback_emotion(const std_msgs::String &msg);

ros::NodeHandleEx<ArduinoHardware> nh;

#define def_log_func(funcname)                          \
void funcname(const char *fmt, ...) {                   \
  char *string;                                         \
  va_list args;                                         \
  va_start(args, fmt);                                  \
  if (0 > vasprintf(&string, fmt, args)) string = NULL; \
  va_end(args);                                         \
  if (string) {                                         \
    nh.funcname(string);                                \
    free(string);                                       \
  }                                                     \
}
def_log_func(logdebug);
def_log_func(loginfo);
def_log_func(logwarn);
def_log_func(logerror);
def_log_func(logfatal);

extern EyeManager eye;

ros::Subscriber<geometry_msgs::Point> sub_point("~look_at", &callback_look_at);
ros::Subscriber<std_msgs::String> sub_eye_status("~eye_status", &callback_emotion);
void callback_look_at(const geometry_msgs::Point &msg)
{
  eye.set_gaze_direction((float)msg.x, (float)msg.y);
}
void callback_emotion(const std_msgs::String &msg)
{
  eye.set_emotion(msg.data);
}

std::string ros_read_asset()
{
  std::map<std::string, EyeAsset>& eye_asset_map = eye.eye_asset_map;
  while (not nh.connected())
  {
    nh.spinOnce();
    delay(1000);
  }
  delay(500);  // wait 0.5 sec before reading asset

  std::ostringstream oss;

  bool mode_right;
  int direction = 1;
  if (nh.getParam("~mode_right", &mode_right)) {
    nh.logdebug(mode_right ? "Read rosparam : mode_right is true" : "mode_right is false");
    oss << "mode_right: " << (mode_right?"True":"False") << "\n";
  } else {
    nh.logwarn("Failed to get mode_right parameter");
  }

  nh.getParam("~direction", &direction);
  nh.logdebug("Read rosparam : direction is %d", direction);
  oss << "direction: " << direction << "\n";

  // get eye_asset_names from rosParam
  std::vector<std::string> eye_asset_names;
  nh.getParam("~eye_asset/names", eye_asset_names);
  oss << "eye_asset_names: ";
  for (auto it = eye_asset_names.begin(); it != eye_asset_names.end(); ++it) {
    oss << *it;
    if (std::next(it) != eye_asset_names.end()) oss << ", ";
  }
  oss << "\n";
  //
  for(auto name: eye_asset_names) {
    char eye_asset_map_key[256];
    // path_upperlid
    for(const std::string& type: {"upperlid", "outline", "iris", "pupil", "reflex"}) {
      std::string path;
      snprintf(eye_asset_map_key, sizeof(eye_asset_map_key), "~eye_asset/%s/path_%s", name.c_str(), type.c_str());
      if (nh.getParam(eye_asset_map_key, path)) {
        oss << "eye_asset_image_path: " << name << ": " << type << ": " << path << "\n";
      }
    }
    // upperlid_position, upperlid_default_pos_x, upperlid_default_pos_y, upperlid_default_theta
    std::vector<int> position;
    snprintf(eye_asset_map_key, sizeof(eye_asset_map_key), "~eye_asset/%s/upperlid_position", name.c_str());
    if (nh.getParam(eye_asset_map_key, position)) {
      oss << "eye_asset_position: " << name << ": upperlid: " << joinVector(position) << "\n";
    }
    for(const std::string& pos: {"default_pos_x", "default_pos_y", "default_theta"}) {
      int data;
      snprintf(eye_asset_map_key, sizeof(eye_asset_map_key), "~eye_asset/%s/upperlid_%s", name.c_str(), pos.c_str());
      if (nh.getParam(eye_asset_map_key, &data)) {
        oss << "eye_asset_" << pos << ": " << name << ": upperlid: " << data << "\n";
      }
    }
  }

  std::string eye_asset_text = oss.str();
  nh.logdebug(eye_asset_text.c_str());
  return eye_asset_text;
}

void setup_ros()
{
#if defined(STAMPC3)
  nh.getHardware()->setBaud(115200);
#endif
  nh.initNode();
  nh.subscribe(sub_point);
  nh.subscribe(sub_eye_status);
  nh.spinOnce();
}

void reconnect_ros(EyeManager &eye)
{
  while (not nh.connected())
  {
    nh.spinOnce();
    delay(1000);
    // when ROS node is re-connected, get rosparam again
    eye.setup_asset(ros_read_asset());
    eye.init();
  }
}
