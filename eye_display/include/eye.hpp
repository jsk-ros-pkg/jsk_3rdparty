#include <math.h>
#include <functional>
#include <vector>
#include <map>
#include <array>
#include <string>
#include <iostream>

#include <Arduino.h>
#include <SPIFFS.h>

#include "util.h"

//  should be implemented in {ros,i2c}_lib.h
extern void logdebug(const char *fmt, ...);
extern void loginfo(const char *fmt, ...);
extern void logwarn(const char *fmt, ...);
extern void logerror(const char *fmt, ...);
extern void logfatal(const char *fmt, ...);

struct EyeAsset {
 std::string name = "default";
 std::string path_outline = "/outline.jpg";    // static
 std::string path_iris = "/iris.jpg";          // set_gaze_direction(x,y) to set position
 std::string path_pupil = "/pupil.jpg";        //  move along with iris
 std::string path_reflex = "/reflex.jpg" ;     //  move along with puppil + random motion
 std::string path_upperlid = "/upperlid.jpg";  // use upperlid_position_map to set y-axis motoin
 std::vector<int> upperlid_position = {0};   // upperlid = motion layer
 int direction = 0;
 bool invert_rl = false;
 int upperlid_pivot_x = 75;
 int upperlid_pivot_y = 139;
 int upperlid_default_pos_x = 75;
 int upperlid_default_pos_y = 7;
 int upperlid_default_theta = 0;
};

#if defined(STAMPS3)
#include <lgfx_round_lcd_stamp_s3.hpp>
#elif defined(STAMPC3)
#include <lgfx_round_lcd_stamp_c3.hpp>
#endif

#if defined(STAMPS3)
#include "ArduinoHWCDCHardware.h"
#elif defined(STAMPC3)
#include "ArduinoHardware.h"
#endif

class EyeManager
{
private:
  LGFX_ROUND_LCD lcd;

  // Spriteを設定
  LGFX_Sprite sprite_eye;
  LGFX_Sprite sprite_outline;
  LGFX_Sprite sprite_iris;
  LGFX_Sprite sprite_pupil;
  LGFX_Sprite sprite_reflex;
  LGFX_Sprite sprite_upperlid;

  float zoom_ratio;
  int image_width = 139;
  int image_height = 139;

  float look_x = 0.0f;
  float look_y = 0.2f;

  int frame = 0;

  void load_eye_images();
  bool draw_image_file(LGFX_Sprite& sprite, const char* filePath);

  std::string current_eye_asset_name;
public:
  std::map<std::string, EyeAsset> eye_asset_map;

  EyeManager();

  void init(const int image_width, const int image_height);
  int setup_asset(std::string eye_asset_text);
  void set_gaze_direction(float look_x, float look_y);
  void set_emotion(const std::string eye_status_name);
  std::string get_emotion();
  int update_emotion();
  void update_look(float dx, float dy,
                   int dx_upperlid, int dy_upperlid, float dtheta_upperlid,
                   float random_scale);
};


EyeManager::EyeManager()
{
  eye_asset_map = {std::map<std::string, EyeAsset>::value_type("default", EyeAsset())};
  current_eye_asset_name = "default";
}

void EyeManager::init(
          const int image_width = 139,
          const int image_height = 139
          )
{
    EyeAsset& current_eye_asset = eye_asset_map[current_eye_asset_name];
    this->image_width = image_width;
    this->image_height = image_height;
    lcd.init();
    lcd.setRotation(current_eye_asset.direction);

    // 目全体を描写するBufferとしてのSpriteを準備
    sprite_eye.createSprite(image_width, image_height);
    sprite_eye.fillScreen(TFT_WHITE);

    // 目の輪郭を描写するSpriteを準備
    sprite_outline.createSprite(image_width, image_height);
    if (current_eye_asset.invert_rl) sprite_outline.setRotation(6);

    // 虹彩を描写するSpriteを準備
    sprite_iris.createSprite(image_width, image_height);
    if (current_eye_asset.invert_rl) sprite_iris.setRotation(6);

    // 瞳孔を描写するSpriteを準備
    sprite_pupil.createSprite(image_width, image_height);
    if (current_eye_asset.invert_rl) sprite_pupil.setRotation(6);

    // 光の反射を描画するSpriteを準備
    sprite_reflex.createSprite(image_width, image_height);
    if (current_eye_asset.invert_rl) sprite_reflex.setRotation(6);

    // 上瞼を描写するSpriteを準備
    sprite_upperlid.createSprite(image_width, image_height);
    if (current_eye_asset.invert_rl) sprite_upperlid.setRotation(6);
    sprite_upperlid.setPivot(current_eye_asset.upperlid_pivot_x, current_eye_asset.upperlid_pivot_y);

    // Load images from default EyeAsset
    //////////////////////////set_emotion(current_eye_asset);

    // lcdを準備
    lcd.setPivot(lcd.width() >> 1, lcd.height() >> 1);
    lcd.fillScreen(TFT_WHITE);

    // zoom率を指定
    zoom_ratio = (float)lcd.width() / image_width;
    float ztmp = (float)lcd.height() / image_height;

    if (zoom_ratio > ztmp)
    {
      zoom_ratio = ztmp;
    }

    set_emotion(current_eye_asset.name);
}

bool EyeManager::draw_image_file(LGFX_Sprite& sprite, const char* filePath)
{
    std::string pathStr(filePath);
    std::string extension = pathStr.substr(pathStr.find_last_of('.') + 1);
    bool ret = false;

    if (extension == "jpg" || extension == "jpeg") {
      logdebug("[%8ld] loading jpeg: %s", millis(), filePath);
      ret = sprite.drawJpgFile(SPIFFS, filePath);
    } else if (extension == "png") {
      logdebug("[%8ld] loading png: %s", millis(), filePath);
      ret = sprite.drawPngFile(SPIFFS, filePath);
    } else {
      logerror("[%8ld] invalid image extension %s", millis(), filePath);
    }
    if (not ret) {
      logerror("[%8ld] Failed to load %s", millis(), filePath);
    }
    return ret;
}

   // 視線方向を変更（値を設定するだけ）
void EyeManager::set_gaze_direction(float look_x, float look_y)
{
    this->look_x = look_x;
    this->look_y = look_y;
    loginfo("[%8ld] Look at (%.1f, %.1f)", millis(), look_x, look_y);
}

// 目の状態を更新する
void EyeManager::load_eye_images()
  {
    EyeAsset& current_eye_asset = eye_asset_map[current_eye_asset_name];
    const char *path_jpg_outline = current_eye_asset.path_outline.c_str();
    const char *path_jpg_iris = current_eye_asset.path_iris.c_str();
    const char *path_jpg_pupil = current_eye_asset.path_pupil.c_str();
    const char *path_jpg_reflex = current_eye_asset.path_reflex.c_str();
    const char *path_jpg_upperlid = current_eye_asset.path_upperlid.c_str();

    lcd.setRotation(current_eye_asset.direction);

    if (path_jpg_outline != NULL) {
        sprite_outline.fillScreen(TFT_WHITE);
        if (not draw_image_file(sprite_outline, path_jpg_outline)) {
            sprite_outline.fillScreen(TFT_WHITE);
        }
    }

    if (path_jpg_iris != NULL) {
        sprite_iris.fillScreen(TFT_WHITE);
        if (not draw_image_file(sprite_iris, path_jpg_iris)) {
            sprite_iris.fillScreen(TFT_WHITE);
        }
    }


    if (path_jpg_pupil != NULL) {
        sprite_pupil.fillScreen(TFT_WHITE);
        if (not draw_image_file(sprite_pupil, path_jpg_pupil)) {
            sprite_pupil.fillScreen(TFT_WHITE);
        }
    }

    if (path_jpg_reflex != NULL) {
        sprite_reflex.fillScreen(TFT_WHITE);
        if (not draw_image_file(sprite_reflex, path_jpg_reflex)) {
            sprite_reflex.fillScreen(TFT_WHITE);
        }
    }

    if (path_jpg_upperlid != NULL) {
        sprite_upperlid.fillScreen(TFT_WHITE);
        if (not draw_image_file(sprite_upperlid, path_jpg_upperlid)) {
            sprite_upperlid.fillScreen(TFT_WHITE);
        }
    }
  }

// 通常の目の描画
void EyeManager::update_look(float dx = 0.0, float dy = 0.0,
          int dx_upperlid = 0.0, int dy_upperlid = 0.0, float dtheta_upperlid = 0.0,
          float random_scale = 5.0)
{
    EyeAsset& current_eye_asset = eye_asset_map[current_eye_asset_name];
    logdebug("[%8ld] [update_look] dx: %.1f, dy: %.1f, dx_upperlid: %d, dy_upperlid: %d, dtheta_upperlid: %d, random_scale: %.1f", millis(), dx, dy, dx_upperlid, dy_upperlid, dtheta_upperlid, random_scale);

    long rx = (int)(random_scale * random(100) / 100);
    long ry = (int)(random_scale * random(100) / 100);

    sprite_eye.clear();
    sprite_eye.fillScreen(TFT_WHITE);
    sprite_outline.pushSprite(&sprite_eye, 0, 0, TFT_WHITE);
    sprite_iris.pushSprite(&sprite_eye, dx, dy, TFT_WHITE);
    sprite_pupil.pushSprite(&sprite_eye, dx, dy, TFT_WHITE); // 瞳孔をランダムに動かす
    sprite_reflex.pushSprite(&sprite_eye, dx + rx, dy + ry, TFT_WHITE); // 光の反射をランダムに動かす
    sprite_upperlid.pushRotateZoom(&sprite_eye,
            current_eye_asset.upperlid_default_pos_x + dx_upperlid,
            current_eye_asset.upperlid_default_pos_y + dy_upperlid,
            current_eye_asset.upperlid_default_theta + dtheta_upperlid,
            1.0, 1.0, TFT_WHITE);

    sprite_eye.pushRotateZoom(&lcd, lcd.width() >> 1, lcd.height() >> 1, 0, zoom_ratio, zoom_ratio);
}

void EyeManager::set_emotion(const std::string eye_status_name) {
  frame = 0;  // reset frame cound for synchronize
  auto it = eye_asset_map.find(eye_status_name);
  if (it == eye_asset_map.end()) {
    logerror("[%8ld] Unknown eye_asset status name %s", millis(), eye_status_name.c_str());
    logerror("[%8ld] possible status are", millis());
    for(auto & eye_asset: eye_asset_map) {
      logerror("[%8ld] ... [%s]", millis(), eye_asset.first.c_str());
    }
    return;
  }
  current_eye_asset_name = it->first;
  loginfo("[%8ld] Status updated: %s", millis(), it->first.c_str());
  load_eye_images();
}

std::string EyeManager::get_emotion() {
  return current_eye_asset_name;
}

int EyeManager::update_emotion() {
    EyeAsset& current_eye_asset = eye_asset_map[current_eye_asset_name];
    float upperlid_y;
    if (current_eye_asset.upperlid_position.size() > 0) {
      upperlid_y = current_eye_asset.upperlid_position[frame % current_eye_asset.upperlid_position.size()];
    }else{
      upperlid_y = 0;
    }
    update_look(look_x, look_y, 0, upperlid_y);  // dx, dy, dx_upperlid, dy_upperlid, dtheta_upperlid
    frame ++;
    return frame;
}

#define check_eye_asset_map_key(name) \
  if (eye_asset_map.find(name) == eye_asset_map.end()) {        \
    logerror("Invalid eye_asset_map_key : [%s]", name.c_str()); \
    logerror("Available keys are...");                          \
    for(auto & eye_asset: eye_asset_map) {                      \
      logerror(" ... [%s]", eye_asset.first.c_str());           \
    }                                                           \
    return -1;                                                  \
  }

int EyeManager::setup_asset(std::string eye_asset_text) {
  static bool mode_right;
  static int direction;

  loginfo("Setup eye asset");
  std::istringstream iss(eye_asset_text);
  std::string message;
  while (std::getline(iss, message)) {
    if ( message.empty() ) { continue; }
    std::string key, value;
    splitKeyValue(message, key, value);
    logdebug("received : %s", message.c_str());
    if ( key == "mode_right" ) {
      if ( value == "True" ) {
        mode_right = true;
      } else if ( value == "False" ) {
        mode_right = false;
      } else {
        logerror("Invalid command for mode_right : %s", value.c_str());
      }
    } else if ( key == "direction" ) {
      direction = std::stoi(value);
    } else if ( key == "eye_asset_names" ) {
      std::list<std::string> eye_asset_names = splitComma(value);
      //
      // initialize eye_asset_map from eye_asset_names
      //
      //std::map<std::string, EyeAsset>& eye_asset_map = this->eye_asset_map;
      for(auto name: eye_asset_names) {
        eye_asset_map[name] = EyeAsset();
        eye_asset_map[name].name = name;
        eye_asset_map[name].direction = direction;
        eye_asset_map[name].invert_rl = not mode_right;
      }
    } else if ( key == "eye_asset_image_path" ) {
      std::string name, type_path, type, path;
      splitKeyValue(value, name, type_path);
      //      check_eye_asset_map_key(name);
      splitKeyValue(type_path, type, path);
      //
      // update eye_asset image map from eye_asset_images
      //
      EyeAsset *asset = &(eye_asset_map[name]);
      if ( type == "outline" ) {
        asset->path_outline = path;
      } else if ( type == "iris" ) {
        asset->path_iris = path;
      } else if ( type == "pupil" ) {
        asset->path_pupil = path;
      } else if ( type == "reflex" ) {
        asset->path_reflex = path;
      } else if ( type == "upperlid" ) {
        asset->path_upperlid = path;
      } else {
        logerror("Invalid eye_asset type_path : %s (%s,%s)", type_path.c_str(), type.c_str(), path.c_str());
        return -1;
      }
    } else if ( key == "eye_asset_position" ) {
      std::string name, type_position, type, position;
      splitKeyValue(value, name, type_position);
      check_eye_asset_map_key(name);
      splitKeyValue(type_position, type, position);
      //
      // update eye_asset image map from eye_asset_upperlid_position
      //
      EyeAsset *asset = &(eye_asset_map[name]);
      std::list<std::string> eye_asset_upperlid_position = splitComma(position);
      if ( type == "upperlid" ) {
        asset->upperlid_position.clear();
        for(std::string pos: eye_asset_upperlid_position) {
          asset->upperlid_position.push_back(std::stoi(pos));
        }
      } else {
        logerror("Invalid eye_asset type_position : %s (%s,%s)", type_position.c_str(), type.c_str(), position.c_str());
        return -1;
      }
    } else if ( key == "eye_asset_default_pos_x" ) {
      std::string name, type_default_pos_x, type, default_pos_x;
      splitKeyValue(value, name, type_default_pos_x);
      check_eye_asset_map_key(name);
      splitKeyValue(type_default_pos_x, type, default_pos_x);
      //
      // update eye_asset image map from eye_asset_default_pos_x
      //
      EyeAsset *asset = &(eye_asset_map[name]);
      if ( type == "upperlid" ) {
        asset->upperlid_default_pos_x = std::stoi(default_pos_x);
      } else {
        logerror("Invalid eye_asset type_default_pos_x : %s (%s,%s)", type_default_pos_x.c_str(), type.c_str(), default_pos_x.c_str());
        return -1;
      }
    } else if ( key == "eye_asset_default_pos_y" ) {
      std::string name, type_default_pos_y, type, default_pos_y;
      splitKeyValue(value, name, type_default_pos_y);
      check_eye_asset_map_key(name);
      splitKeyValue(type_default_pos_y, type, default_pos_y);
      //
      // update eye_asset image map from eye_asset_default_pos_y
      //
      EyeAsset *asset = &(eye_asset_map[name]);
      if ( type == "upperlid" ) {
        asset->upperlid_default_pos_y = std::stoi(default_pos_y);
      } else {
        logerror("Invalid eye_asset type_default_pos_y : %s (%s,%s)", type_default_pos_y.c_str(), type.c_str(), default_pos_y.c_str());
        return -1;
      }
    } else if ( key == "eye_asset_default_theta" ) {
      std::string name, type_default_theta, type, default_theta;
      splitKeyValue(value, name, type_default_theta);
      check_eye_asset_map_key(name);
      splitKeyValue(type_default_theta, type, default_theta);
      //
      // update eye_asset image map from eye_asset_default_theta
      //
      EyeAsset *asset = &(eye_asset_map[name]);
      if ( type == "upperlid" ) {
        asset->upperlid_default_theta = std::stoi(default_theta);
      } else {
        logerror("Invalid eye_asset type_default_theta : %s (%s,%s)", type_default_theta.c_str(), type.c_str(), default_theta.c_str());
        return -1;
      }
    } else {
      logerror("Invlalid command : %s (key : %s, value : %s)", message.c_str(), key.c_str(), value.c_str());
      return -1;
    }
  }
  // display map data
  // to show this message,
  //  call rosservice call eye_display/set_logger_level rosout DEBUG
  // or
  //  roslaunch launch file with debug:=true
  for(auto& it: eye_asset_map) {
    std::string name = it.first;
    EyeAsset &eye_asset = eye_asset_map[name];
    loginfo("[%s]", name.c_str());
    loginfo("     mode_right : %s", eye_asset.invert_rl?"True":"False");
    loginfo("      direction : %d", eye_asset.direction);
    loginfo("  outline image : %s", eye_asset.path_outline.c_str());
    loginfo("     iris image : %s", eye_asset.path_iris.c_str());
    loginfo("    pupil image : %s", eye_asset.path_pupil.c_str());
    loginfo("   reflex image : %s", eye_asset.path_reflex.c_str());
    loginfo(" upperlid image : %s", eye_asset.path_upperlid.c_str());
    std::ostringstream oss;
    std::copy(eye_asset.upperlid_position.begin(), eye_asset.upperlid_position.end(), std::ostream_iterator<float>(oss, ", "));
    std::string result = oss.str(); result.pop_back(); result.pop_back();  // remove last ","
    loginfo(" upperlid_positions: %s", result.c_str());
    loginfo(" upperlid_default_pos_x : %d", eye_asset.upperlid_default_pos_x);
    loginfo(" upperlid_default_pos_y : %d", eye_asset.upperlid_default_pos_y);
    loginfo(" upperlid_default_theta : %d", eye_asset.upperlid_default_theta);
  }
  // eyeの初期化
  if ( !eye_asset_map.empty() ) {
    set_emotion(eye_asset_map.begin()->first);
  } else {
    logwarn("Faile to initialize emotion, use default asset");
  }
  return 0;
}

#if !defined(USE_I2C) && !defined(USE_ROS) // sample code for eye asset without ROS/I2C
void logdebug(const char *fmt, ...) { }
void loginfo(const char *fmt, ...) { }
void logwarn(const char *fmt, ...) { }
void logerror(const char *fmt, ...) { }
void logfatal(const char *fmt, ...) { }
#endif
