#ifndef __VT_DUMMY_H__
#define __VT_DUMMY_H__
extern "C" {
  void VT_UNLOADTTS_JPN(int);
  int VT_LOADTTS_JPN(int, int, char*, char*);
  int VT_TextToFile_JPN(int, char *, char *, int, int, int, int, int, int, int);
  void VT_GetTTSInfo_JPN(int , char *, void *, int);
  int VT_LOADTTS_SUCCESS = 0;
  int VT_FILE_API_SUCCESS = 0;
  int VT_FILE_API_FMT_S16PCM_WAVE = 4; // https://pastebin.com/9LeCr2HN
}

enum
  {
    VT_BUILD_DATE              =  0,
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
    VT_MAX_SENT_PAUSE          = 20,
    VT_DEF_SENT_PAUSE          = 21,
    VT_MIN_SENT_PAUSE      = 22,
    VT_DB_BUILD_DATE       = 23,
    VT_MAX_COMMA_PAUSE         = 24,
    VT_DEF_COMMA_PAUSE         = 25,
    VT_MIN_COMMA_PAUSE         = 26,
    VT_MAX_SYMBOL_OPEN_PAUSE           = 27,
    VT_DEF_SYMBOL_OPEN_PAUSE           = 28,
    VT_MIN_SYMBOL_OPEN_PAUSE           = 29,
    VT_MAX_SYMBOL_CLOSE_PAUSE          = 30,
    VT_DEF_SYMBOL_CLOSE_PAUSE          = 31,
    VT_MIN_SYMBOL_CLOSE_PAUSE          = 32,
  };

#endif //__VT_DUMMY_H__
