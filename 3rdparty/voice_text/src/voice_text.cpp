/*
 * voice_text_server.cpp
 * Author: Yuki Furuta <furushchev@jsk.imi.i.u-tokyo.ac.jp>
 */

#include <fstream>
#include <cstdlib>
#include <string>

#include <boost/filesystem.hpp>
#include <boost/thread.hpp>

// ROS
#include <ros/ros.h>
#include <ros/package.h>
#include <dynamic_reconfigure/server.h>
#include <voice_text/VoiceTextConfig.h>
#include <voice_text/TextToSpeech.h>

#include "vt_wrapper.h"

namespace fs = boost::filesystem;

bool search_vt_lib_path(const std::string& db_path, std::string& lib_path)
{
  fs::path search_path(db_path);
  if (fs::exists(search_path)) {
    fs::recursive_directory_iterator it_end;
    std::string re = "";
#ifdef __i386__
    re += "x86_32/";
#else
    re += "x86_64/";
#endif
    re += "RAMIO/libvt_jpn.so";
    for (fs::recursive_directory_iterator it(search_path); it != it_end; ++it)
    {
      if (fs::is_regular_file(it->path()))
      {
        std::string p = it->path().string();
        int offset = (int)p.size() - re.size();
        if (offset >= 0 && p.find(re, offset) != std::string::npos)
        {
          lib_path = p;
          return true;
        }
      }
    }
  }

  return false;
}

class VoiceText {
public:
  typedef voice_text::VoiceTextConfig Config;

  VoiceText() : nh_(), pnh_("~"), dyn_srv_(pnh_) {
    pkg_path_ = ros::package::getPath("voice_text");

    std::string lib_path, db_path, license_path;
    pnh_.param<std::string>("db_path", db_path, "/usr/vt/sayaka/M16");
    pnh_.param<std::string>("license_path", license_path, "");
    pnh_.param<std::string>("lib_path", lib_path, "");

    /*
      If param 'lib_path' is not specified,
      try to find using 'db_path' as a hint.
    */
    if (lib_path.empty())
    {
      if (!search_vt_lib_path(db_path, lib_path))
        ROS_ERROR_STREAM("lib_path is not specified and cannot be found.");
    }

    dynamic_reconfigure::Server<Config>::CallbackType f =
      boost::bind(&VoiceText::config_callback, this, _1, _2);
    dyn_srv_.setCallback(f);

    init_library(lib_path, db_path, license_path);

    srv_ = nh_.advertiseService("text_to_speech", &VoiceText::text_to_speech, this);
    ROS_INFO_STREAM("Advertised service 'text_to_speech'");
  }

  ~VoiceText() {}

  void init_library(const std::string& lib_path,
                    const std::string& db_path,
                    const std::string& license_path)
  {
    if (lib_path.empty()) {
      ROS_ERROR_STREAM("Library path is empty");
      return;
    } else if (!fs::exists(fs::path(lib_path))) {
      ROS_ERROR_STREAM("Library does not exists: " << lib_path);
      return;
    }

    ROS_INFO_STREAM("Found VT Library: " << lib_path);
    bool ok = vt_.load(lib_path);
    if (!ok) {
      ROS_ERROR_STREAM("Failed to load VT library");
      return;
    }

    TTSInfo info;
    if (license_path.empty()) {
      ROS_INFO_STREAM("License path is empty. Default path is used.");
    } else if (!fs::exists(fs::path(license_path))) {
      ROS_ERROR_STREAM("License file does not exists: " << license_path);
      return;
    }
    ok = vt_.GetTTSInfo_JPN(license_path, info);
    if (!ok) {
      ROS_INFO_STREAM("Loaded VT library");
      return;
    }
    ROS_INFO_STREAM("Loaded VT library (Build Date: " << info.build_date << ", Sample Rate: " << info.sampling_frequency << ")");

    if (db_path.empty()) {
      ROS_ERROR_STREAM("DB path is empty");
      return;
    } else if (!fs::exists(fs::path(db_path))) {
      ROS_ERROR_STREAM("DB directory does not exists: " << db_path);
      return;
    }
    ok = vt_.LOADTTS_JPN(-1, db_path, license_path);
    if (!ok) {
      ROS_ERROR_STREAM("Failed to load TTS");
      return;
    }
    ROS_INFO_STREAM("Loaded TTS");
  }

  void config_callback(Config &config, uint32_t level) {
    boost::mutex::scoped_lock lock(mutex_);
    config_ = config;
  }

  bool text_to_speech(voice_text::TextToSpeech::Request  &req,
                      voice_text::TextToSpeech::Response &res) {
    boost::mutex::scoped_lock lock(mutex_);

    try {
      // check library is loaded
      if (!vt_.isLoaded())
        throw std::runtime_error("VT library not initialized");

      // load text from file
      if (!fs::exists(fs::path(req.text_path)))
        throw std::runtime_error("text file " + req.text_path + " not found");

      std::ifstream ifs(req.text_path.c_str());
      std::string text = "", line = "";
      while (ifs && std::getline(ifs, line)) {
        text += line;
      }

      bool ok = vt_.TextToFile_JPN(text, req.wave_path,
                                   -1,
                                   config_.pitch,
                                   config_.speed,
                                   config_.volume,
                                   config_.pause,
                                   0, 0);
      if (!ok)
        throw std::runtime_error("Failed to text to file");
    } catch (const std::runtime_error& e) {
      ROS_ERROR_STREAM(e.what());
      // copy error wave file
      fs::path wav_path = fs::path(pkg_path_) / fs::path("data/error.wav");
      fs::path dest_path = fs::path(req.wave_path);
      if (fs::exists(wav_path) &&
          fs::exists(dest_path.parent_path())) {
        fs::copy_file(wav_path, fs::path(req.wave_path),
                      fs::copy_option::overwrite_if_exists);
      }
      res.ok = false;
    }
    res.ok = true;
    return true;
  }

  ros::NodeHandle nh_, pnh_;
  boost::mutex mutex_;
  dynamic_reconfigure::Server<Config> dyn_srv_;
  Config config_;
  ros::ServiceServer srv_;
  std::string pkg_path_;
  VTWrapper vt_;
};


int main(int argc, char** argv) {

  ros::init(argc, argv, "voice_text");

  VoiceText vt;

  ros::spin();

  return 0;
}
