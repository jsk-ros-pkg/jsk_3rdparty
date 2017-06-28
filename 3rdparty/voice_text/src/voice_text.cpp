/*
 * voice_text_server.cpp
 * Author: Yuki Furuta <furushchev@jsk.imi.i.u-tokyo.ac.jp>
 */

#include <fstream>
#include <cstdlib>
#include <sstream>
#include <string>

#include <boost/filesystem.hpp>
#include <boost/thread.hpp>

// ROS
#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include <voice_text/VoiceTextConfig.h>
#include <voice_text/TextToSpeech.h>

// VoiceText
#include "/usr/vt/sayaka/M16/inc/vt_jpn.h"

namespace fs = boost::filesystem;


class VoiceText {
public:
  typedef voice_text::VoiceTextConfig Config;

  VoiceText() : nh_(), pnh_("~"), db_path_(""), license_path_(""), dyn_srv_(pnh_) {
    pnh_.param<std::string>("db_path", db_path_, "/usr/vt/sayaka/M16");
    pnh_.param<std::string>("license_path", license_path_, "");

    dynamic_reconfigure::Server<Config>::CallbackType f =
      boost::bind(&VoiceText::config_callback, this, _1, _2);
    dyn_srv_.setCallback(f);
  }

  ~VoiceText() {
    if (initialized_) {
      VT_UNLOADTTS_JPN(-1);
    }
  }

  void config_callback(Config &config, uint32_t level) {
    boost::mutex::scoped_lock lock(mutex_);
    config_ = config;
  }

  bool initialize() {
    // initialize voice text
    int ret = -1;
    char* db_path_char = (char*)calloc(std::strlen(db_path_.c_str())+1, sizeof(char));
    std::strcpy(db_path_char, db_path_.c_str());
    char* license_path_char = NULL;
    if (!license_path_.empty()) {
      license_path_char = (char*)calloc(std::strlen(license_path_.c_str())+1, sizeof(char));
      std::strcpy(license_path_char, license_path_.c_str());
    }
    ret = VT_LOADTTS_JPN((int)NULL, -1, db_path_char, license_path_char);
    free(db_path_char);
    if (!license_path_.empty()) free(license_path_char);
    if (ret != VT_LOADTTS_SUCCESS) {
      ROS_FATAL("Failed to load TTS engine (code %d)", ret);
      return false;
    }

    // advertise service
    srv_ = nh_.advertiseService("text_to_speech", &VoiceText::text_to_speech, this);

    ROS_INFO_STREAM("Advertised service text_to_speech");

    return true;
  }

  bool text_to_speech(voice_text::TextToSpeech::Request  &req,
                      voice_text::TextToSpeech::Response &res) {
    boost::mutex::scoped_lock lock(mutex_);
    // load text from file
    if (!fs::exists(fs::path(req.text_path))) {
      ROS_ERROR_STREAM("text file " << req.text_path << " not found");
      res.ok = false;
      return true;
    }
    std::ifstream ifs(req.text_path.c_str());
    std::string text = "", line = "";
    while (ifs && std::getline(ifs, line)) {
      text += line;
    }
    char* text_char = (char*)calloc(std::strlen(text.c_str())+1, sizeof(char));
    std::strcpy(text_char, text.c_str());

    char* wave_char = (char*)calloc(std::strlen(req.wave_path.c_str())+1, sizeof(char));
    std::strcpy(wave_char, req.wave_path.c_str());

    int ret = VT_TextToFile_JPN(VT_FILE_API_FMT_S16PCM_WAVE,
                                text_char,
                                wave_char,
                                -1,
                                config_.pitch,
                                config_.speed,
                                config_.volume,
                                config_.pause,
                                -1, -1);

    free(text_char);
    free(wave_char);

    if (ret != VT_FILE_API_SUCCESS) {
      ROS_ERROR("Failed to execute tts: (code: %d)", ret);
      res.ok = false;
      return true;
    }

    res.ok = true;
    return true;
  }

  ros::NodeHandle nh_, pnh_;
  boost::mutex mutex_;
  dynamic_reconfigure::Server<Config> dyn_srv_;
  Config config_;
  ros::ServiceServer srv_;
  bool initialized_;
  std::string db_path_, license_path_;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "voice_text");

  VoiceText vt;
  if (!vt.initialize()) {
    return 1;
  };

  ros::spin();

  return 0;
}
