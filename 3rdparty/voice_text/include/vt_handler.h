/*
 * vt_handler.h
 * Author: Yoshiki Obinata <obinata@jsk.imi.i.u-tokyo.ac.jp>
 */

#ifndef VT_HANDLER_H_
#define VT_HANDLER_H_

#include <dlfcn.h>
#include <glob.h>
#include <string>
#include <boost/filesystem.hpp>

// logging
#include <ros/ros.h>

#include "vt_jpn.h"
#include "vtapi.h"

#define VT_ROOT "/usr/vt/*/*"
#define PATH_MAX 1024

#if __x86_64__ || __ppc64__
#define ENV64
#else
#define ENV32
#endif

typedef enum VT_TYPE{
NO_VT,
VT_SDK,
VT_API
} VT_Types;

namespace fs = boost::filesystem;

class VTHandler{
    public:
        VTHandler(const std::string license_path, const std::string db_path);
        ~VTHandler();
        bool VTH_TextToFile(const int pitch, const int speed, const int volume, const int pause,
                            const std::string text, const std::string wave_path);

    private:
        void* dl_handle;
        VT_Types vt_type;

        // define handle
        bool LoadSym();

        // Load symbols
        // Related to VoiceText SDK
        std::vector<char*> VTSDK_func_ = {
        "VT_LOADTTS_JPN",
        "VT_UNLOADTTS_JPN",
        "VT_GetTTSInfo_JPN",
        "VT_TextToFile_JPN"
        };

        // Related to ReadSpeaker API
        std::vector<char*> VTAPI_func_ = {
        "VTAPI_Init",
        "VTAPI_CreateHandle",
        "VTAPI_SetLicenseFolder",
        "VTAPI_GetEngine",
        "VTAPI_SetEngineHandle",
        "VTAPI_SetAttr",
        "VTAPI_SetOutputFile",
        "VTAPI_TextToFile",
        "VTAPI_GetLastErrorInfo",
        "VTAPI_ReleaseHandle",
        "VTAPI_UnloadEngine",
        "VTAPI_Exit"
        };

        // symbol map
        std::map<char*, void*> VTSDK_s_map_;
        std::map<char*, void*> VTAPI_s_map_;

        // Load Functions
        // Related to VoiceText SDK
        short (*VT_LOADTTS_JPN)(HWND, int, char*, char*);
        void (*VT_UNLOADTTS_JPN)(int);
        int (*VT_GetTTSInfo_JPN)(int, char*, void*, int);
        short (*VT_TextToFile_JPN)(int, char*, char*, int, int, int, int, int, int, int);

        // Related to ReadSpeaker API
        int (*VTAPI_Init)(char*);
        VTAPI_HANDLE (*VTAPI_CreateHandle)();
        void (*VTAPI_SetLicenseFolder)(char*);
        VTAPI_ENGINE_HANDLE (*VTAPI_GetEngine)(char*, char*);
        int (*VTAPI_SetEngineHandle)(VTAPI_HANDLE, VTAPI_ENGINE_HANDLE);
        int (*VTAPI_SetAttr)(VTAPI_HANDLE, int, int);
        int (*VTAPI_SetOutputFile)(VTAPI_HANDLE, char*, int);
        int (*VTAPI_TextToFile)(VTAPI_HANDLE, void*, int, int);
        VTAPI_ERRS_INFO* (*VTAPI_GetLastErrorInfo)(VTAPI_HANDLE);
        void (*VTAPI_ReleaseHandle)(VTAPI_HANDLE);
        int (*VTAPI_UnloadEngine)(VTAPI_ENGINE_HANDLE);
        void (*VTAPI_Exit)();

        // ReadSpeaker API handler
        VTAPI_HANDLE hVTAPI;
        VTAPI_ENGINE_HANDLE hEngine;
};


#endif // VT_HANDLER_H_
