// -*- mode: C++ -*-
/*
 * vt_wrapper.h
 * Author: Yuki Furuta <furushchev@jsk.imi.i.u-tokyo.ac.jp>
 */


#ifndef VT_WRAPPER_H__
#define VT_WRAPPER_H__

#include <boost/filesystem.hpp>
#include <boost/shared_ptr.hpp>
#include <cstdlib>
#include <Poco/SharedLibrary.h>
#include <ros/ros.h>
#include <string>

namespace fs = boost::filesystem;

struct TTSInfo {
  std::string build_date, db_directory, db_build_date;
  int verify_code, max_channel, load_success_code;
  int max_speaker, def_speaker;
  int sampling_frequency;
};

class VTWrapper {
 public:
  VTWrapper(){}
  ~VTWrapper()
  {
    unload();
  }

  bool load(std::string lib_path)
  {
    // check if library exists
    if (!fs::exists(fs::path(lib_path))) return false;
    try
    {
      handle_.reset(new Poco::SharedLibrary(lib_path));
      return true;
    } catch (const Poco::LibraryAlreadyLoadedException &e) {
      ROS_WARN_STREAM("Library already loaded");
      return true;
    } catch(const std::exception &e) {
      ROS_ERROR_STREAM("Failed to load library: " << e.what());
      return false;
    }
  }

  bool isLoaded()
  {
    return handle_ && handle_->isLoaded();
  }

  void unload()
  {
    if (isLoaded())
      handle_->unload();
  }

  bool LOADTTS_JPN(const int nSpeakerID,
                   const std::string& db_path,
                   const std::string& licensefile)
  {
    if (!handle_) return false;
    short (*func)(int, int, char*, char*) = NULL;
    try {
      func = (short (*)(int, int, char*, char*))handle_->getSymbol("VT_LOADTTS_JPN");
    } catch (const Poco::NotFoundException &e) {
      ROS_ERROR_STREAM("Failed to load TTS " << e.what());
      return false;
    }

    char* db_path_c = new char[db_path.size()+1];
    std::strcpy(db_path_c, db_path.c_str());

    char* license_c = NULL;
    if (!licensefile.empty()) {
      license_c = new char[licensefile.size()+1];
      std::strcpy(license_c, licensefile.c_str());
    }

    short ret = func((int)NULL, nSpeakerID, db_path_c, license_c);
    delete[] db_path_c;
    if (!licensefile.empty()) delete[] license_c;
    return ret == 0;
  }

  bool UNLOADTTS_JPN(const int nSpeakerID)
  {
    if (!handle_) return false;
    void (*func)(int) = NULL;
    try {
      func = (void (*)(int))handle_->getSymbol("VT_UNLOADTTS_JPN");
    } catch (const Poco::NotFoundException &e) {
      ROS_ERROR_STREAM("Failed to unload TTS: " << e.what());
      return false;
    }

    func(nSpeakerID);
    return true;
  }

  bool LOAD_UserDict_JPN(const int dictidx,
                         const std::string& filename)
  {
    if (!handle_) return false;
    short (*func)(int, char*) = NULL;
    try {
      func = (short (*)(int, char*))handle_->getSymbol("VT_LOAD_UserDict_JPN");
    } catch (const Poco::NotFoundException &e) {
      ROS_ERROR_STREAM("Failed to load UserDict: " << e.what());
      return false;
    }

    char* filename_c = new char[filename.size()+1];
    std::strcpy(filename_c, filename.c_str());

    short ret = func(dictidx, filename_c);
    delete[] filename_c;

    return ret == 1;
  }

  bool UNLOAD_UserDict_JPN(const int dictidx)
  {
    if (!handle_) return false;
    short (*func)(int) = NULL;
    try {
      func = (short (*)(int))handle_->getSymbol("VT_UNLOAD_UserDict_JPN");
    } catch (const Poco::NotFoundException &e) {
      ROS_ERROR_STREAM("Failed to unload UserDict: " << e.what());
      return false;
    }

    short ret = func(dictidx);

    return ret == 1;
  }

  bool TextToFile_JPN(const std::string& tts_text,
                      const std::string& filename,
                      const int nSpeakerID,
                      const int pitch,
                      const int speed,
                      const int volume,
                      const int pause,
                      const int dictidx,
                      const int texttype)
  {
    if (!handle_) return false;
    const int fmt = 4; /* = VT_FILE_API_FMT_S16PCM_WAVE */
    short (*func)(int, char*, char*, int, int, int, int, int, int, int) = NULL;
    try {
      func = (short (*)(int, char*, char*, int, int, int, int, int, int, int))handle_->getSymbol("VT_TextToFile_JPN");
    } catch (const Poco::NotFoundException &e) {
      ROS_ERROR_STREAM("Failed to TextToFile: " << e.what());
      return false;
    }

    char* tts_text_c = new char[tts_text.size()+1];
    std::strcpy(tts_text_c, tts_text.c_str());

    char* filename_c = new char[filename.size()+1];
    std::strcpy(filename_c, filename.c_str());

    short ret = func(fmt, tts_text_c, filename_c, nSpeakerID,
                     pitch, speed, volume, pause,
                     dictidx, texttype);
    delete[] tts_text_c;
    delete[] filename_c;
    return ret == 1;
  }

  bool GetTTSInfo_JPN(const std::string& licensefile,
                      TTSInfo& ttsinfo)
  {
    if (!handle_) return false;
    void (*func)(int, char*, void*, int) = NULL;
    try {
      func = (void (*)(int, char*, void*, int))handle_->getSymbol("VT_GetTTSInfo_JPN");
    } catch (const Poco::NotFoundException &e) {
      ROS_ERROR_STREAM("Failed to get TTS info: " << e.what());
      return false;
    }

    char* license_c = NULL;
    if (!licensefile.empty()) {
      license_c = new char[licensefile.size()+1];
      std::strcpy(license_c, licensefile.c_str());
    }

    char* buff_c = new char[256];

    /* VT_BUILD_DATE = 0 */
    func(0, license_c, (void*)buff_c, 256);
    ttsinfo.build_date = std::string(buff_c);

    /* VT_VERIFY_CODE         =  1 */
    func(1, license_c, (void*)&ttsinfo.verify_code, sizeof(int));

    /* VT_MAX_CHANNEL         =  2 */
    func(2, license_c, (void*)&ttsinfo.max_channel, sizeof(int));

    /* VT_DB_DIRECTORY        =  3 */
    func(3, license_c, (void*)buff_c, 256);
    ttsinfo.db_directory = std::string(buff_c);

    /* VT_LOAD_SUCCESS_CODE   =  4 */
    func(4, license_c, (void*)&ttsinfo.load_success_code, sizeof(int));

    /* VT_MAX_SPEAKER         =  5 */
    func(5, license_c, (void*)&ttsinfo.max_speaker, sizeof(int));

    /* VT_DEF_SPEAKER         =  6 */
    func(6, license_c, (void*)&ttsinfo.def_speaker, sizeof(int));

    /* VT_SAMPLING_FREQUENCY  = 10 */
    func(10, license_c, (void*)&ttsinfo.sampling_frequency, sizeof(int));

    if (!licensefile.empty()) delete[] license_c;
    delete[] buff_c;
    return true;
  }

  boost::shared_ptr<Poco::SharedLibrary> handle_;
};


#endif // VT_WRAPPER_H__
