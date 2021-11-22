/*
 * voice_text.cpp
 * Author: Yuki Furuta <furushchev@jsk.imi.i.u-tokyo.ac.jp>, Yoshiki Obinata <obinata@jsk.imi.i.u-tokyo.ac.jp>
 */

#include <fstream>
#include <cstdlib>
#include <sstream>
#include <string>

#include <glob.h>
#include <boost/filesystem.hpp>
#include <boost/thread.hpp>

// ROS
#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include <voice_text/VoiceTextConfig.h>
#include <voice_text/TextToSpeech.h>

#include "vt_handler.h"

namespace fs = boost::filesystem;

class VoiceText {
public:
  typedef voice_text::VoiceTextConfig Config;

  VoiceText() : nh_(), pnh_("~"), db_path_(""), license_path_(""), dyn_srv_(pnh_){
    pnh_.param<std::string>("db_path", db_path_, "");
    pnh_.setParam("db_path", db_path_);  // for backward compatibility (db_path is usually set previously)
    pnh_.param<std::string>("license_path", license_path_, "");

    dynamic_reconfigure::Server<Config>::CallbackType f =
      boost::bind(&VoiceText::config_callback, this, _1, _2);
    dyn_srv_.setCallback(f);

    h_vt_ = (new VTHandler(license_path_, db_path_));

    srv_ = nh_.advertiseService("text_to_speech", &VoiceText::text_to_speech, this);
    ROS_INFO("Advertised service text_to_speech\n");
  }

    ~VoiceText(){
      delete h_vt_;
    }

  void config_callback(Config &config, uint32_t level) {
    boost::mutex::scoped_lock lock(mutex_);
    config_ = config;
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
    res.ok = h_vt_->VTH_TextToFile(config_.pitch, config_.speed, config_.volume, config_.pause,
                                   text, req.wave_path);
    return true;
  }

  ros::NodeHandle nh_, pnh_;
  boost::mutex mutex_;
  dynamic_reconfigure::Server<Config> dyn_srv_;
  Config config_;
  ros::ServiceServer srv_;
  bool initialized_;
  std::string db_path_, license_path_;
  VTHandler* h_vt_;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "voice_text");

  VoiceText vt;

  ros::spin();

  return 0;
}
