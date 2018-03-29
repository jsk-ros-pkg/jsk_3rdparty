#ifndef __VT_DUMMY_H__
#define __VT_DUMMY_H__
extern "C" {
  void VT_UNLOADTTS_JPN(int);
  int VT_LOADTTS_JPN(int, int, char*, char*);
  int VT_TextToFile_JPN(int, char *, char *, int, int, int, int, int, int, int);
  int VT_LOADTTS_SUCCESS = 0;
  int VT_FILE_API_SUCCESS = 0;
  int VT_FILE_API_FMT_S16PCM_WAVE = 4; // https://pastebin.com/9LeCr2HN
}
#endif //__VT_DUMMY_H__
