#pragma once

#include <iterator>
#include <sstream>
#include <Wire.h>
#include <WireSlave.h>

constexpr int SDA_PIN = 8;
constexpr int SCL_PIN = 9;
constexpr int I2C_SLAVE_ADDR = 0x42;

class EyeManagerIO : public EyeManager {
public:
  EyeManagerIO() : EyeManager() {}

#define def_eye_manager_log_func(funcname)                \
  void funcname(const char *fmt, ...) override {          \
    char *string;                                         \
    va_list args;                                         \
    va_start(args, fmt);                                  \
    if (0 > vasprintf(&string, fmt, args)) string = NULL; \
    va_end(args);                                         \
    if (string) {                                         \
      Serial.print("[");                                  \
      Serial.print(#funcname);                            \
      Serial.print("] ");                                 \
      Serial.println(string);                             \
      free(string);                                       \
    }                                                     \
  }
  def_eye_manager_log_func(logdebug);
  def_eye_manager_log_func(loginfo);
  def_eye_manager_log_func(logwarn);
  def_eye_manager_log_func(logerror);
  def_eye_manager_log_func(logfatal);
};

#include <list>

std::string trim(const std::string& s) {
  auto start = std::find_if_not(s.begin(), s.end(), ::isspace);
  auto end = std::find_if_not(s.rbegin(), s.rend(), ::isspace).base();
  if (start >= end) return "";
  return std::string(start, end);
}

std::list<std::string> splitComma(const std::string& input) {
  std::list<std::string> result;
  std::stringstream ss(input);
  std::string item;

  while (std::getline(ss, item, ',')) {
    result.push_back(trim(item));
  }

  // just in case where last element is blank
  if (!input.empty() && input.back() == ',') {
    result.push_back("");  // add null element
  }

  return result;
}

bool splitKeyValue(const std::string& message, std::string& key, std::string& value) {
  size_t colonPos = message.find(':');
  if (colonPos == std::string::npos) {
    return false;  // failed if ":" is not found
  }

  // trim key
  size_t keyStart = 0;
  size_t keyEnd = colonPos;
  while (keyEnd > keyStart && std::isspace(message[keyEnd - 1])) --keyEnd;

  // trim value
  size_t valStart = colonPos + 1;
  while (valStart < message.size() && std::isspace(message[valStart])) ++valStart;
  size_t valEnd = message.size();
  while (valEnd > valStart && std::isspace(message[valEnd - 1])) --valEnd;

  key = message.substr(keyStart, keyEnd - keyStart);
  value = message.substr(valStart, valEnd - valStart);
  return true;
}

bool parseXY(const std::string& input, double& x, double& y) {
  // remove spaces
  std::string cleaned;
  for (char c : input) {
    // convert camma to spaces
    if (c == ',') {
      cleaned += ' ';
    } else {
      cleaned += c;
    }
  }

  std::istringstream iss(cleaned);
  return static_cast<bool>(iss >> x >> y);  // return true if there are x and y
}

extern EyeManagerIO eye;
bool mode_right = true;
int direction = 1;

void receiveEvent(int howMany) {
  std::string message;
  while (0 < WireSlave.available()) {
    char c = WireSlave.read();  // receive byte as a character
    message += c;
  }
  eye.logdebug("I2C received : %s", message.c_str());

  std::string key, value;
  splitKeyValue(message, key, value);
  if ( key == "mode_right" ) {
    if ( value == "True" ) {
      mode_right = true;
    } else if ( value == "False" ) {
      mode_right = false;
    } else {
      eye.logerror("Invalid command for mode_right : %s", value.c_str());
    }
  } else if ( key == "direction" ) {
    direction = std::stoi(value);
  } else if ( key  == "eye_status" ) {
    std::map<std::string, EyeAsset>& eye_asset_map = eye.eye_asset_map;
    // for (const auto& pair : eye_asset_map) {
    //   const std::string& name = pair.first;
    //   const EyeAsset& eye_asset = pair.second;
    //   eye.logdebug("[%s]", name.c_str());
    //   eye.logdebug("  outline image : %s", eye_asset.path_outline.c_str());
    //   eye.logdebug("     iris image : %s", eye_asset.path_iris.c_str());
    //   eye.logdebug("    pupil image : %s", eye_asset.path_pupil.c_str());
    //   eye.logdebug("   reflex image : %s", eye_asset.path_reflex.c_str());
    //   eye.logdebug(" upperlid image : %s", eye_asset.path_upperlid.c_str());
    //   std::ostringstream oss;
    //   std::copy(eye_asset.upperlid_position.begin(), eye_asset.upperlid_position.end(), std::ostream_iterator<int>(oss, ", "));
    //   std::string result = oss.str(); result.pop_back(); result.pop_back();  // remove last ","
    //   eye.logdebug(" upperlid_positions: %s", result.c_str());
    //   eye.logdebug(" upperlid_default_pos_x : %d", eye_asset.upperlid_default_pos_x);
    //   eye.logdebug(" upperlid_default_pos_y : %d", eye_asset.upperlid_default_pos_y);
    //   eye.logdebug(" upperlid_default_theta : %d", eye_asset.upperlid_default_theta);
      eye.set_emotion(value);
    // }
  } else if ( key  == "look_at") {
    double x, y;
    parseXY(value, x, y);
    eye.set_gaze_direction(x, y);
  } else if ( key == "eye_asset_names" ) {
    std::list<std::string> eye_asset_names = splitComma(value);
    //
    // initialize eye_asset_map from eye_asset_names
    //
    std::map<std::string, EyeAsset>& eye_asset_map = eye.eye_asset_map;
    for(auto name: eye_asset_names) {
      eye_asset_map[name] = EyeAsset();
      eye_asset_map[name].name = name;
      eye_asset_map[name].direction = direction;
      eye_asset_map[name].invert_rl = not mode_right;
    }
  } else if ( key == "eye_asset_images" ) {
    std::string name, images;
    splitKeyValue(value, name, images);
    //
    // update eye_asset image map from eye_asset_images
    //
    std::list<std::string> eye_asset_images = splitComma(images);
    if ( eye_asset_images.size() != 5 ) {
      eye.logerror("Invalid eye_asset_images : %s", images.c_str());
      return;
    }
    //
    std::map<std::string, EyeAsset>& eye_asset_map = eye.eye_asset_map;
    if (eye_asset_map.find(name) == eye_asset_map.end()) {
      eye.logerror("Invalid eye_asset_map_key : %s", name.c_str());
      return;
    }
    EyeAsset *asset = &(eye_asset_map[name]);
    auto it = eye_asset_images.begin();
    if ( ! it->empty() ) asset->path_outline  = *it; it++;
    if ( ! it->empty() ) asset->path_iris     = *it; it++;
    if ( ! it->empty() ) asset->path_pupil    = *it; it++;
    if ( ! it->empty() ) asset->path_reflex   = *it; it++;
    if ( ! it->empty() ) asset->path_upperlid = *it;
    //show status message
    EyeAsset &eye_asset = eye_asset_map[name];
    eye.logdebug("[%s]", name.c_str());
    eye.logdebug("  outline image : %s", eye_asset.path_outline.c_str());
    eye.logdebug("     iris image : %s", eye_asset.path_iris.c_str());
    eye.logdebug("    pupil image : %s", eye_asset.path_pupil.c_str());
    eye.logdebug("   reflex image : %s", eye_asset.path_reflex.c_str());
    eye.logdebug(" upperlid image : %s", eye_asset.path_upperlid.c_str());
  } else if ( key == "eye_asset_upperlid_position" ) {
    std::string name, positions;
    splitKeyValue(value, name, positions);
    std::map<std::string, EyeAsset>& eye_asset_map = eye.eye_asset_map;
    if (eye_asset_map.find(name) == eye_asset_map.end()) {
      eye.logerror("Invalid eye_asset_map_key : %s", name.c_str());
      return;
    }
    //
    // update eye_asset image map from eye_asset_upperlid_position
    //
    std::list<std::string> eye_asset_upperlid_positions = splitComma(positions);
    EyeAsset *asset = &(eye_asset_map[name]);
    asset->upperlid_position.clear();
    for(std::string pos: eye_asset_upperlid_positions) {
      asset->upperlid_position.push_back(std::stoi(pos));
    }
    //show status message
    EyeAsset &eye_asset = eye_asset_map[name];
    eye.logdebug("[%s]", name.c_str());
    std::ostringstream oss;
    std::copy(eye_asset.upperlid_position.begin(), eye_asset.upperlid_position.end(), std::ostream_iterator<int>(oss, ", "));
    std::string result = oss.str(); result.pop_back(); result.pop_back();  // remove last ","
    eye.logdebug(" upperlid_positions: %s", result.c_str());
  } else if ( key == "eye_asset_upperlid_default" ) {
    std::string name, defaults;
    splitKeyValue(value, name, defaults);
    std::map<std::string, EyeAsset>& eye_asset_map = eye.eye_asset_map;
    if (eye_asset_map.find(name) == eye_asset_map.end()) {
      eye.logerror("Invalid eye_asset_map_key : %s", name.c_str());
      return;
    }
    //
    // update eye_asset image map from eye_asset_upperlid_position
    //
    std::list<std::string> eye_asset_upperlid_defaults = splitComma(defaults);
    EyeAsset *asset = &(eye_asset_map[name]);
    auto it = eye_asset_upperlid_defaults.begin();
    if ( ! it->empty() ) asset->upperlid_default_pos_x = std::stoi(*it); it++;
    if ( ! it->empty() ) asset->upperlid_default_pos_y = std::stoi(*it); it++;
    if ( ! it->empty() ) asset->upperlid_default_theta = std::stoi(*it);
    //show status message
    EyeAsset &eye_asset = eye_asset_map[name];
    eye.logdebug("[%s]", name.c_str());
    eye.logdebug(" upperlid_default_pos_x : %d", eye_asset.upperlid_default_pos_x);
    eye.logdebug(" upperlid_default_pos_y : %d", eye_asset.upperlid_default_pos_y);
    eye.logdebug(" upperlid_default_theta : %d", eye_asset.upperlid_default_theta);
  } else {
    eye.logerror("Invlalid command : %s (key : %s, value : %s)", message.c_str(), key.c_str(), value.c_str());
  }
  return;
}

void I2CTask(void *parameter) {
  bool success = WireSlave.begin(SDA_PIN, SCL_PIN, I2C_SLAVE_ADDR);

  Serial.println("I2C slave start");
  if (!success) {
    Serial.println("I2C slave init failed");
    while (1) delay(100);
  }
  WireSlave.onReceive(receiveEvent);
  while (true) {
    WireSlave.update();
    delay(1);  // let I2C and other ESP32 peripherals interrupts work
  }
}

void setup_asset(EyeManager& eye)  // returns initial status
{
  std::map<std::string, EyeAsset>& eye_asset_map = eye.eye_asset_map;
  eye_asset_map["normal"] = EyeAsset();
  eye_asset_map["normal"].name = "normal";
  eye_asset_map["normal"].upperlid_position = {9};
  eye_asset_map["normal"].direction = 1;
#if defined(EYE_RIGHT)
  eye_asset_map["normal"].direction = true;
#else
  eye_asset_map["normal"].invert_rl = false;
#endif

  eye_asset_map["blink"] = EyeAsset();
  eye_asset_map["blink"].name = "blink";
  eye_asset_map["blink"].upperlid_position = {9, 9, 130, 130, 9, 9};
  eye_asset_map["blink"].direction = 1;
#if defined(EYE_RIGHT)
  eye_asset_map["blink"].direction = true;
#else
  eye_asset_map["blink"].invert_rl = false;
#endif

  eye.set_emotion("blink");
}

void setup_i2c()
{
  xTaskCreatePinnedToCore(I2CTask, "I2C Task", 4096, NULL, 24, NULL, 0);
}
