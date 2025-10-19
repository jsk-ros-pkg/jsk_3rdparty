/*
 * vt_handler.cpp
 * Author: Yoshiki Obinata <obinata@jsk.imi.i.u-tokyo.ac.jp>
 */

#include "vt_handler.h"

VTHandler::VTHandler(const std::string license_path, const std::string db_path,
                     const bool enable_custom_engine_info,
                     const std::string speaker,
                     const std::string type,
                     const int speaker_id,
                     const std::string lang,
                     const std::string gender,
                     const int code_page,
                     const std::string iso_code,
                     const std::string vendor,
                     const int sampling_rate){
  glob_t sdk_old_gbuf_{}, sdk_new_gbuf_{}, api_gbuf_{};
  std::string lib_file_;
  char *dl_err_, *db_path_char_, *license_path_char_;
  bool sym_status_;
  int ret_;

  // Locate libraries
#if defined(__x86_64__) || defined(__ppc64__)
  ROS_INFO("Loading x86_64 binary");
  glob(("/usr/vt/*/*/bin/x86_64/RAMIO/libvt_jpn.so"), 0, NULL, &sdk_old_gbuf_); // e.g., /usr/vt/sayaka/M16/bin/x86_64/RAMIO/libvt_jpn.so
  glob(("/usr/vt/*/*/bin/LINUX64_GLIBC3/RAMIO/libvt_jpn.so"), 0, NULL, &sdk_new_gbuf_); // e.g., /usr/vt/risa/H16/bin/LINUX64_GLIBC3/RAMIO/libvt_jpn.so
#elif defined(__aarch64__) || defined(_M_ARM64)
  ROS_INFO("Loading aarch64 binary");
#elif defined(i386) || defined(__i386__) || defined(__i386) || defined(_M_IX86)
  ROS_INFO("Loading i386 binary");
  glob(("/usr/vt/*/*/bin/x86_32/RAMIO/libvt_jpn.so"), 0, NULL, &sdk_old_gbuf_); // e.g., /usr/vt/sayaka/M16/bin/x86_32/RAMIO/libvt_jpn.so
  glob(("/usr/vt/*/*/bin/LINUX32_GLIBC3/RAMIO/libvt_jpn.so"), 0, NULL, &sdk_new_gbuf_); // e.g., /usr/vt/risa/H16/bin/LINUX32_GLIBC3/RAMIO/libvt_jpn.so
#else
  ROS_FATAL("Your architecture is not supported");
  return;
#endif
  glob(("/usr/vt/*/*/bin/libvtapi.so"), 0, NULL, &api_gbuf_); // e.g., /usr/vt/hikari/D16/bin/libvtapi.so

  if(sdk_old_gbuf_.gl_pathc > 0){
    this->vt_type = VT_SDK;
    lib_file_ = sdk_old_gbuf_.gl_pathv[0];
  }else if(sdk_new_gbuf_.gl_pathc > 0){
    this->vt_type = VT_SDK;
    lib_file_ = sdk_new_gbuf_.gl_pathv[0];
  }else if(api_gbuf_.gl_pathc > 0){
    this->vt_type = VT_API;
    lib_file_ = api_gbuf_.gl_pathv[0];
  }else{
    this->vt_type = NO_VT;
  }

  globfree(&sdk_old_gbuf_);
  globfree(&sdk_new_gbuf_);
  globfree(&api_gbuf_);

  // Load libraries
  if(this->vt_type != NO_VT){
    ROS_INFO("Opening %s ", lib_file_.c_str());
    this->dl_handle = dlopen(lib_file_.c_str(), RTLD_NOW);
    if(this->dl_handle == NULL){
      dl_err_ = dlerror();
      ROS_FATAL_STREAM("Error occured when opening VoiceText or ReadSpeaker libraries " <<
                       dl_err_);
      return;
    }
  }else{
    ROS_FATAL("No VoiceText or ReadSpeaker libraries have found");
    return;
  }

  // Load symbols
  sym_status_ = LoadSym();
  if(!sym_status_){
    return;
  }

  // Initialize VT Handler
  // db_path is for backward compatibility
  db_path_char_ = (char*)calloc(std::strlen(db_path.c_str())+1, sizeof(char));
  std::strcpy(db_path_char_, db_path.c_str());

  // Load license file
  license_path_char_ = NULL;
  if(!license_path.empty()){
    license_path_char_ = (char*)calloc(std::strlen(license_path.c_str())+1, sizeof(char));
    std::strcpy(license_path_char_, license_path.c_str());
  }else{
    ROS_FATAL("Please set license file");
    return;
  }

  // Load license file
  if(this->vt_type == VT_SDK){
    ret_ = VT_LOADTTS_JPN((int)NULL, -1, db_path_char_, license_path_char_);
    if(ret_ != VT_LOADTTS_SUCCESS){
      ROS_FATAL("[VoiceText SDK] Failed to load TTS engine (code %d)", ret_);
      return;
    }
    VT_GetTTSInfo_JPN(VT_VERIFY_CODE, NULL, &ret_, sizeof(int));
    if (ret_ != 0) {
      ROS_FATAL_STREAM("[VoiceText SDK] Verification failed (VT_VERIFY_CODE " << ret_ << ")");
      return;
    }
  }else if(this->vt_type == VT_API){
    fs::path p_ = lib_file_;
    std::vector<std::string> elements_;
    std::string lib_path_, speaker_, type_;
    char *lib_path_char_, *speaker_char_, *type_char_;

    // Get ReadSpeaker library directory
    lib_path_ = p_.parent_path().string();
    lib_path_char_ = (char*)calloc(std::strlen(lib_path_.c_str())+1, sizeof(char));
    std::strcpy(lib_path_char_, lib_path_.c_str());

    // Get speaker and type
    for(auto& part_ : p_){
      elements_.push_back(part_.string());
    }
    speaker_ = elements_.at(3);
    speaker_char_ = (char*)calloc(std::strlen(speaker_.c_str())+1, sizeof(char));
    std::strcpy(speaker_char_, speaker_.c_str());
    type_ = elements_.at(4);
    type_char_ = (char*)calloc(std::strlen(type_.c_str())+1, sizeof(char));
    std::strcpy(type_char_, type_.c_str());

    ROS_INFO("Loading libary at: %s", lib_path_char_);
    VTAPI_Init(lib_path_char_);
    this->hVTAPI = VTAPI_CreateHandle();
    if(this->hVTAPI == 0){
      ROS_FATAL("[ReadSpeaker API] Failed to create API handler. STATUS: %s", VTAPI_GetLastErrorInfo(this->hVTAPI)->szMsg);
      return;
    }
    VTAPI_SetLicenseFolder(license_path_char_);
    // Load engine
    if (enable_custom_engine_info){
        ROS_INFO("[ReadSpeaker API] Add new custom engine");
        char *lang_char_, *gender_char_, *iso_code_char_, *vendor_char_;
        lang_char_ = (char*)calloc(std::strlen(lang.c_str())+1, sizeof(char));
        std::strcpy(lang_char_, lang.c_str());
        gender_char_ = (char*)calloc(std::strlen(gender.c_str())+1, sizeof(char));
        std::strcpy(gender_char_, gender.c_str());
        iso_code_char_ = (char*)calloc(std::strlen(iso_code.c_str())+1, sizeof(char));
        std::strcpy(iso_code_char_, iso_code.c_str());
        vendor_char_ = (char*)calloc(std::strlen(vendor.c_str())+1, sizeof(char));
        std::strcpy(vendor_char_, vendor.c_str());
        this->hEngine = VTAPI_AddNewEngineInfo(speaker_char_,
                                               type_char_,
                                               speaker_id,
                                               db_path_char_,
                                               lang_char_,
                                               gender_char_,
                                               code_page,
                                               iso_code_char_,
                                               vendor_char_,
                                               sampling_rate);
        free(lang_char_);
        free(gender_char_);
        free(iso_code_char_);
        free(vendor_char_);
    }
    this->hEngine = VTAPI_GetEngine(speaker_char_, type_char_);
    ret_ = VTAPI_SetEngineHandle(this->hVTAPI, this->hEngine);
    if(ret_ < VTAPI_SUCCESS){
      ROS_FATAL("[ReadSpeaker API] Failed to create engine handler. CODE: %d, MESSAGE: %s ",
                ret_, VTAPI_GetLastErrorInfo(this->hVTAPI)->szMsg);
      return;
    }

    free(lib_path_char_);
    free(speaker_char_);
    free(type_char_);
  }

  free(db_path_char_);
  if (!license_path.empty()) free(license_path_char_);

}

VTHandler::~VTHandler(){
  if(this->dl_handle != NULL){
    // TODO release handle before close dl
    dlclose(this->dl_handle);
  }
}

bool VTHandler::LoadSym(){
  const char* dl_err_;
  if(vt_type == VT_SDK){
    ROS_INFO("Found VoiceText SDK");
    // load symbol
    for(auto& itr: VTSDK_func_){
      VTSDK_s_map_[itr] = dlsym(this->dl_handle, itr);
      dl_err_ = dlerror();
      if(dl_err_ != NULL){
        ROS_FATAL_STREAM("Failed to load VoiceText libraries. STATUS: "
                         << dl_err_);
        dlclose(this->dl_handle);
        return false;
        break;
      }
    }
    // cast
    VT_LOADTTS_JPN = reinterpret_cast<short(*)(HWND, int, char*, char*)>(VTSDK_s_map_.at("VT_LOADTTS_JPN"));
    VT_UNLOADTTS_JPN = reinterpret_cast<void(*)(int)>(VTSDK_s_map_.at("VT_UNLOADTTS_JPN"));
    VT_GetTTSInfo_JPN = reinterpret_cast<int(*)(int, char*, void*, int)>(VTSDK_s_map_.at("VT_GetTTSInfo_JPN"));
    VT_TextToFile_JPN = reinterpret_cast<short(*)(int, char*, char*, int, int, int, int, int, int, int)>(VTSDK_s_map_.at("VT_TextToFile_JPN"));
  }else if(vt_type == VT_API){
    ROS_INFO("Found ReadSpeaker API");
    // load symbol
    for(auto& itr: VTAPI_func_){
      VTAPI_s_map_[itr] = dlsym(this->dl_handle, itr);
      dl_err_ = dlerror();
      if(dl_err_ != NULL){
        ROS_FATAL_STREAM("Failed to load ReadSpeaker libraries. STATUS: "
                         << dl_err_);
        dlclose(this->dl_handle);
        return false;
        break;
      }
    }
    // cast
    VTAPI_Init = reinterpret_cast<int(*)(char*)>(VTAPI_s_map_.at("VTAPI_Init"));
    VTAPI_CreateHandle = reinterpret_cast<VTAPI_HANDLE(*)()>(VTAPI_s_map_.at("VTAPI_CreateHandle"));
    VTAPI_SetLicenseFolder = reinterpret_cast<void(*)(char*)>(VTAPI_s_map_.at("VTAPI_SetLicenseFolder"));
    VTAPI_AddNewEngineInfo = reinterpret_cast<VTAPI_ENGINE_HANDLE(*)(char*, char*, int, char*, char*, char*, int, char*, char*, int)>(VTAPI_s_map_.at("VTAPI_AddNewEngineInfo"));
    VTAPI_UpdateInstalledEngine = reinterpret_cast<int(*)()>(VTAPI_s_map_.at("VTAPI_UpdateInstalledEngine"));
    VTAPI_GetEngine = reinterpret_cast<VTAPI_ENGINE_HANDLE(*)(char*, char*)>(VTAPI_s_map_.at("VTAPI_GetEngine"));
    VTAPI_SetEngineHandle = reinterpret_cast<int(*)(VTAPI_HANDLE, VTAPI_ENGINE_HANDLE)>(VTAPI_s_map_.at("VTAPI_SetEngineHandle"));
    VTAPI_SetAttr = reinterpret_cast<int(*)(VTAPI_HANDLE, int, int)>(VTAPI_s_map_.at("VTAPI_SetAttr"));
    VTAPI_SetOutputFile = reinterpret_cast<int(*)(VTAPI_HANDLE, char*, int)>(VTAPI_s_map_.at("VTAPI_SetOutputFile"));
    VTAPI_TextToFile = reinterpret_cast<int(*)(VTAPI_HANDLE, void*, int, int)>(VTAPI_s_map_.at("VTAPI_TextToFile"));
    VTAPI_GetLastErrorInfo = reinterpret_cast<VTAPI_ERRS_INFO*(*)(VTAPI_HANDLE)>(VTAPI_s_map_.at("VTAPI_GetLastErrorInfo"));
    VTAPI_ReleaseHandle = reinterpret_cast<void(*)(VTAPI_HANDLE)>(VTAPI_s_map_.at("VTAPI_ReleaseHandle"));
    VTAPI_UnloadEngine = reinterpret_cast<int(*)(VTAPI_ENGINE_HANDLE)>(VTAPI_s_map_.at("VTAPI_UnloadEngine"));
    VTAPI_Exit = reinterpret_cast<void(*)()>(VTAPI_s_map_.at("VTAPI_Exit"));
  }
  return true;
}

bool VTHandler::VTH_TextToFile(const int pitch, const int speed, const int volume, const int pause,
                               const std::string text, const std::string wave_path){
  char *text_char_, *wave_path_char_;
  int ret_;
  bool success_ = true;

  text_char_ = (char*)calloc(std::strlen(text.c_str())+1, sizeof(char));
  std::strcpy(text_char_, text.c_str());
  wave_path_char_ = (char*)calloc(std::strlen(wave_path.c_str())+1, sizeof(char));
  std::strcpy(wave_path_char_, wave_path.c_str());

  if(this->vt_type == VT_SDK){
    ret_ = VT_TextToFile_JPN(VT_FILE_API_FMT_S16PCM_WAVE,
                             text_char_,
                             wave_path_char_,
                             -1,
                             pitch,
                             speed,
                             volume,
                             pause,
                             -1, -1);
    if(ret_ != VT_FILE_API_SUCCESS){
      ROS_ERROR("[VoiceText SDK] Failed to execute TTS (code %d)", ret_);
      success_ = false;
    }
  }else if(this->vt_type == VT_API){
    VTAPI_SetAttr(this->hVTAPI, ATTR_PITCH, pitch);
    VTAPI_SetAttr(this->hVTAPI, ATTR_SPEED, speed);
    VTAPI_SetAttr(this->hVTAPI, ATTR_VOLUME, volume);
    VTAPI_SetAttr(this->hVTAPI, ATTR_PAUSE, pause);
    ret_ = VTAPI_SetOutputFile(this->hVTAPI, wave_path_char_, FORMAT_16PCM_WAV);
    if(ret_ != VTAPI_SUCCESS){
      ROS_ERROR("[ReadSpeaker API] Failed to execute VTAPI_SetOutputFile. STATUS: %s",
                VTAPI_GetLastErrorInfo(this->hVTAPI)->szMsg);
      success_ = false;
    }
    ret_ = VTAPI_TextToFile(this->hVTAPI, text_char_, -1, TEXT_FORMAT_DEFAULT);
    if(ret_ != VTAPI_SUCCESS){
      ROS_ERROR("[ReadSpeaker API] Failed to execute VTAPI_TextToFile. STATUS: %s",
                VTAPI_GetLastErrorInfo(this->hVTAPI)->szMsg);
      success_ = false;
    }

    free(text_char_);
    free(wave_path_char_);
  }
  return success_;
}
