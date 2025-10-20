/*
 * vt_handler.h
 * Author: Yoshiki Obinata <obinata@jsk.imi.i.u-tokyo.ac.jp>
 */

#ifndef VT_HANDLER_H_
#define VT_HANDLER_H_

#include <cstdlib>
#include <dlfcn.h>
#include <glob.h>
#include <string>
#include <boost/filesystem.hpp>

// logging
#include <ros/ros.h>

// struct from vt_jpn.h
typedef	int HWND;
#define		VT_LOADTTS_SUCCESS							0
#define		VT_LOADTTS_ERROR_CONFLICT_DBPATH			1
#define		VT_LOADTTS_ERROR_TTS_STRUCTURE				2
#define		VT_LOADTTS_ERROR_TAGGER						3
#define		VT_LOADTTS_ERROR_BREAK_INDEX				4
#define		VT_LOADTTS_ERROR_TPP_DICT					5
#define		VT_LOADTTS_ERROR_TABLE						6
#define		VT_LOADTTS_ERROR_UNIT_INDEX					7
#define		VT_LOADTTS_ERROR_PROSODY_DB					8
#define		VT_LOADTTS_ERROR_PCM_DB						9
#define		VT_LOADTTS_ERROR_PM_DB						10
#define		VT_LOADTTS_ERROR_UNKNOWN					11
#define		VT_FILE_API_SUCCESS						(1)
#define		VT_FILE_API_ERROR_INVALID_FORMAT		(-1)
#define		VT_FILE_API_ERROR_CREATE_THREAD			(-2)
#define		VT_FILE_API_ERROR_NULL_TEXT				(-3)
#define		VT_FILE_API_ERROR_EMPTY_TEXT			(-4)
#define		VT_FILE_API_ERROR_DB_NOT_LOADED			(-5)
#define		VT_FILE_API_ERROR_OUT_FILE_OPEN			(-6)
#define		VT_FILE_API_ERROR_UNKNOWN				(-7)
enum
	{
		VT_BUILD_DATE		   =  0,
		VT_VERIFY_CODE         =  1,
		VT_MAX_CHANNEL         =  2,
		VT_DB_DIRECTORY        =  3,
		VT_LOAD_SUCCESS_CODE   =  4,
		VT_MAX_SPEAKER         =  5,
		VT_DEF_SPEAKER         =  6,
		VT_CODEPAGE            =  7,
		VT_DB_ACCESS_MODE      =  8,
		VT_FIXED_POINT_SUPPORT =  9,
		VT_SAMPLING_FREQUENCY  = 10,
		VT_MAX_PITCH_RATE      = 11,
		VT_DEF_PITCH_RATE      = 12,
		VT_MIN_PITCH_RATE      = 13,
		VT_MAX_SPEED_RATE      = 14,
		VT_DEF_SPEED_RATE      = 15,
		VT_MIN_SPEED_RATE      = 16,
		VT_MAX_VOLUME          = 17,
		VT_DEF_VOLUME          = 18,
		VT_MIN_VOLUME          = 19,
		VT_MAX_SENT_PAUSE	   = 20,
		VT_DEF_SENT_PAUSE	   = 21,
		VT_MIN_SENT_PAUSE      = 22,
		VT_DB_BUILD_DATE       = 23,
		VT_MAX_COMMA_PAUSE	   = 24,
		VT_DEF_COMMA_PAUSE	   = 25,
		VT_MIN_COMMA_PAUSE	   = 26,
		VT_MAX_SYMBOL_OPEN_PAUSE	   = 27,
		VT_DEF_SYMBOL_OPEN_PAUSE	   = 28,
		VT_MIN_SYMBOL_OPEN_PAUSE	   = 29,
		VT_MAX_SYMBOL_CLOSE_PAUSE	   = 30,
		VT_DEF_SYMBOL_CLOSE_PAUSE	   = 31,
		VT_MIN_SYMBOL_CLOSE_PAUSE	   = 32,
	};
enum
    {
        VT_FILE_API_FMT_S16PCM		= 0,
        VT_FILE_API_FMT_ALAW		= 1,
        VT_FILE_API_FMT_MULAW		= 2,
        VT_FILE_API_FMT_DADPCM		= 3,
        VT_FILE_API_FMT_S16PCM_WAVE	= 4,
        VT_FILE_API_FMT_U08PCM_WAVE	= 5,
        //	VT_FILE_API_FMT_IMA_WAVE	= 6, /* not supported! */
        VT_FILE_API_FMT_ALAW_WAVE	= 7,
        VT_FILE_API_FMT_MULAW_WAVE	= 8,
        VT_FILE_API_FMT_MULAW_AU	= 9,
    };

// struct from vtapi.h
#define MAX_ERR_MSG				512
typedef struct VOICE_INFO* VTAPI_HANDLE;
typedef struct ENGINE_INFO* VTAPI_ENGINE_HANDLE;
typedef struct _vtapi_errs
{
	int nErr;
	char szMsg[MAX_ERR_MSG];
} VTAPI_ERRS_INFO;
#define		VTAPI_SUCCESS						 0
typedef enum ATTRFLAGS
{
    ATTR_PITCH = 0,
    ATTR_SPEED = 1,
    ATTR_VOLUME = 2,
	ATTR_PAUSE = 3,
	ATTR_DICTIDX = 4,
#if 1 //0 //!defined(USE_NEW_ENGINE_SETTING) //maintain for compatibility
	ATTR_COMMAPAUSE = 5,
	ATTR_PARENTHESISNUM = 6,
#if 0 // sjchang 2021-04-01 Remove
	ATTR_EMPHASISFACTOR = 7,
#endif
#endif
	ATTR_MODE_SSML = 8,

#if 1 //defined(USE_OUTPUT_MARGIN_EACH_REQUEST) // with // sjchang 2021-05-13 Add for v3.12.7.x
	ATTR_FRONT_MARGIN = 9,
	ATTR_REAR_MARGIN = 10,
	ATTR_MIDDLE_MARGIN = 11,
#endif

#if 1 // defined(USE_USERCONFIG) // sjchang 2022-08-05 Add for UserConfig
	ATTR_CONFIGIDX = 12,
#endif

	ATTR_MAX
} Attr_Flags;
typedef enum OUTPUTFORMAT
{
	FORMAT_16PCM = 0,
	FORMAT_8PCM = 1,
	FORMAT_ALAW_PCM = 2,
	FORMAT_MULAW_PCM = 3,
	FORMAT_ADPCM_PCM = 4,
	FORMAT_16PCM_WAV = 5,
	FORMAT_8PCM_WAV = 6,
	FORMAT_ALAW_WAV = 7,
	FORMAT_MULAW_WAV = 8,

	FORMAT_MAX
} Output_Format;
typedef enum TEXTTYPES
{
	TEXT_FORMAT_DEFAULT = 0, // multibyte
	TEXT_FORMAT_UNICODE = 1,
	TEXT_FORMAT_UTF8 = 2,
	TEXT_FORMAT_JEITA = 4,
	TEXT_FORMAT_JEITA_PLUS = 8,
	TEXT_FORMAT_BIG5 = 16,

	TEXT_CONTENT_NORMAL = 0,
	TEXT_CONTENT_SSML = 128,

	TEXT_INPUTTYPE_STRING = 0, // string input
	TEXT_INPUTTYPE_FILE = 1024, // filename input
} Text_Types;

typedef enum VT_TYPE{
NO_VT,
VT_SDK,
VT_API
} VT_Types;

namespace fs = boost::filesystem;

class VTHandler{
    public:
        VTHandler(const std::string license_path, const std::string db_path,
                  const bool enable_custom_engine_info = false,
                  const std::string speaker = "risa",
                  const std::string type = "g16",
                  const int speaker_id = 308,
                  const std::string lang = "jpn",
                  const std::string gender = "F",
                  const int code_page = 932,
                  const std::string iso_code = "ja_JP",
                  const std::string vendor = "vw",
                  const int sampling_rate = 8000);
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
        "VTAPI_AddNewEngineInfo",
        "VTAPI_UpdateInstalledEngine",
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
        VTAPI_ENGINE_HANDLE (*VTAPI_AddNewEngineInfo)(char*, char*, int, char*, char*, char*, int, char*, char*, int);
        int (*VTAPI_UpdateInstalledEngine)();
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
