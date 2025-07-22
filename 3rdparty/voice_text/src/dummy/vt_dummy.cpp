#include "vt_dummy.h"
#include <stdio.h>

void VT_UNLOADTTS_JPN(int) {};
int VT_LOADTTS_JPN(int, int, char*, char*) {
  fprintf(stderr, "LOADING DUMMY VT_LOADTTS_JPN\n");
  fprintf(stderr, "You need to install voice_text application\n");
  fprintf(stderr, "exiting....\n");
  return -1;
};
int VT_TextToFile_JPN(int, char *, char *, int, int, int, int, int, int, int) {};

void VT_GetTTSInfo_JPN(int , char *, void *, int) {};
