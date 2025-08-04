#pragma once

#include <iterator>
#include <sstream>

#include "node_handle_ex.h"  // #include "ros/node_handle.h"
#include "geometry_msgs/Point.h"
#include "std_msgs/String.h"

void callback_look_at(const geometry_msgs::Point &msg);
void callback_emotion(const std_msgs::String &msg);

ros::NodeHandleEx<ArduinoHardware> nh;

bool ros_was_connected = false;
bool ros_now_connected = false;
#define def_log_func(funcname)                          \
void funcname(const char *fmt, ...) {                   \
  char *string;                                         \
  va_list args;                                         \
  if (!ros_now_connected) {return;}                     \
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
  eye.set_gaze_direction((float)msg.x, (float)msg.y, (float)msg.z);
}
void callback_emotion(const std_msgs::String &msg)
{
  eye.set_emotion(msg.data);
}

std::string ros_read_asset()
{
  std::map<std::string, EyeAsset>& eye_asset_map = eye.eye_asset_map;
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
    std::string name = *it;
    oss << name;
    if (std::next(it) != eye_asset_names.end()) oss << ", ";
  }
  oss << "\n";
  //
  for(auto name: eye_asset_names) {
    char eye_asset_map_key[256];

    // path
    std::vector<std::string> eye_types;
    snprintf(eye_asset_map_key, sizeof(eye_asset_map_key), "~eye_asset_%s_eye_types", name.c_str());
    nh.getParam(eye_asset_map_key, eye_types);
    for(const std::string& type: eye_types) {
      std::string path;
      snprintf(eye_asset_map_key, sizeof(eye_asset_map_key), "~eye_asset/%s/path_%s", name.c_str(), type.c_str());
      if (nh.getParam(eye_asset_map_key, path)) {
        oss << "eye_asset_image_path: " << name << ": " << type << ": " << path << "\n";
      }
    }

    // position
    std::vector<std::string> eye_positions;
    snprintf(eye_asset_map_key, sizeof(eye_asset_map_key), "~eye_asset_%s_positions", name.c_str());
    nh.getParam(eye_asset_map_key, eye_positions);
    for(const std::string& eye_position: eye_positions) {
      if (eye_position == "NONE") continue;
      std::vector<int> position;
      snprintf(eye_asset_map_key, sizeof(eye_asset_map_key), "~eye_asset/%s/%s", name.c_str(), eye_position.c_str());
      if (nh.getParam(eye_asset_map_key, position)) {
        size_t pos = eye_position.find('_');
        if ( pos != std::string::npos ) {
          std::string eye_type = eye_position.substr(0, pos);
          std::string position_type = eye_position.substr(pos+1);
          oss << "eye_asset_" << position_type << ": " << name << ": " << eye_type << ": " << joinVector(position) << "\n";
        } else {
          logerror("Invalid param name : %s", eye_position);
        }
      }
    }

    // rotation
    std::vector<std::string> eye_rotations;
    snprintf(eye_asset_map_key, sizeof(eye_asset_map_key), "~eye_asset_%s_rotations", name.c_str());
    nh.getParam(eye_asset_map_key, eye_rotations);
    for(const std::string& eye_rotation: eye_rotations) {
      if (eye_rotation == "NONE") continue;
      std::vector<int> rotation;
      snprintf(eye_asset_map_key, sizeof(eye_asset_map_key), "~eye_asset/%s/%s", name.c_str(), eye_rotation.c_str());
      if (nh.getParam(eye_asset_map_key, rotation)) {
        size_t pos = eye_rotation.find('_');
        if ( pos != std::string::npos ) {
          std::string eye_type = eye_rotation.substr(0, pos);
          std::string rotation_type = eye_rotation.substr(pos+1);
          oss << "eye_asset_" << rotation_type << ": " << name << ": " << eye_type << ": " << joinVector(rotation) << "\n";
        } else {
          logerror("Invalid param name : %s", eye_rotation);
        }
      }
    }

    // default
    std::vector<std::string> eye_defaults;
    snprintf(eye_asset_map_key, sizeof(eye_asset_map_key), "~eye_asset_%s_defaults", name.c_str());
    nh.getParam(eye_asset_map_key, eye_defaults);
    for(const std::string& eye_default: eye_defaults) {
      if (eye_default == "NONE") continue;
      if (eye_default.find("_default_zoom") != std::string::npos) {
	std::vector<float> defaults;
	snprintf(eye_asset_map_key, sizeof(eye_asset_map_key), "~eye_asset/%s/%s", name.c_str(), eye_default.c_str());
	if (nh.getParam(eye_asset_map_key, defaults)) {
	  size_t pos = eye_default.find('_');
	  if ( pos != std::string::npos ) {
	    std::string eye_type = eye_default.substr(0, pos);
	    std::string default_type = eye_default.substr(pos+1);
	    oss << "eye_asset_" << default_type << ": " << name << ": " << eye_type << ": " << joinVector(defaults) << "\n";
	  } else {
	    logerror("Invalid param name : %s", eye_default);
	  }
	}
      } else {
	std::vector<int> defaults;
	snprintf(eye_asset_map_key, sizeof(eye_asset_map_key), "~eye_asset/%s/%s", name.c_str(), eye_default.c_str());
	if (nh.getParam(eye_asset_map_key, defaults)) {
	  size_t pos = eye_default.find('_');
	  if ( pos != std::string::npos ) {
	    std::string eye_type = eye_default.substr(0, pos);
	    std::string default_type = eye_default.substr(pos+1);
	    oss << "eye_asset_" << default_type << ": " << name << ": " << eye_type << ": " << joinVector(defaults) << "\n";
	  } else {
	    logerror("Invalid param name : %s", eye_default);
	  }
	}
      }
    }

    // zoom
    std::vector<std::string> eye_zooms;
    snprintf(eye_asset_map_key, sizeof(eye_asset_map_key), "~eye_asset_%s_zooms", name.c_str());
    nh.getParam(eye_asset_map_key, eye_zooms);
    for(const std::string& eye_zoom: eye_zooms) {
      if (eye_zoom == "NONE") continue;
      std::vector<float> zooms;
      snprintf(eye_asset_map_key, sizeof(eye_asset_map_key), "~eye_asset/%s/%s", name.c_str(), eye_zoom.c_str());
      if (nh.getParam(eye_asset_map_key, zooms)) {
        size_t pos = eye_zoom.find('_');
        if ( pos != std::string::npos ) {
          std::string eye_type = eye_zoom.substr(0, pos);
          std::string zoom_type = eye_zoom.substr(pos+1);
          oss << "eye_asset_" << zoom_type << ": " << name << ": " << eye_type << ": " << joinVector(zooms) << "\n";
        } else {
          logerror("Invalid param name : %s", eye_zoom);
        }
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
  ros_now_connected = nh.connected();
  if (ros_now_connected && !ros_was_connected) {
    loginfo("ROS reconnected, initializing eye assets");
    // when ROS node is re-connected, get rosparam again
    eye.setup_asset(ros_read_asset());
  }
  ros_was_connected = ros_now_connected;
}
