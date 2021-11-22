/*
* Copyright (c) 2004 Voiceware Co., Ltd., All rights reserved.
*
* VoiceText
*/

#ifndef VT_JPN_H
#define VT_JPN_H

#if defined(__cplusplus)
	extern "C" {
#endif

#if !defined(VT_BASIC_DEFINE)
	#if defined(WIN32)
		#if !defined(_DllMode)
			#define _DllMode(_type_)		__declspec( dllimport ) _type_
		#endif
	#else
		#if !defined(_DllMode)
			#define		_DllMode(_type_)		extern _type_
		#endif
		typedef		int						HWND;
	#endif
#endif




/*===========================================================================*/
/* Text format (used in texttype) */
#if !defined(VT_BASIC_DEFINE)
	#if !defined(VT_TEXT_FMT_PLAIN_TEXT)
		#define	VT_TEXT_FMT_PLAIN_TEXT						0
	#endif

	#if !defined(VT_TEXT_FMT_JEITA)
		#define	VT_TEXT_FMT_JEITA							4
	#endif

	#if !defined(VT_TEXT_FMT_JEITA_PLUS)
		#define	VT_TEXT_FMT_JEITA_PLUS						6
	#endif
#endif



/*===========================================================================*/
/* LOAD & UNLOAD */
#if !defined(VT_BASIC_DEFINE)
	/* Return Value */
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
#endif

_DllMode(short)	VT_LOADTTS_JPN(HWND hWnd, int nSpeakerID, char *db_path, char *licensefile);
_DllMode(void)	VT_UNLOADTTS_JPN(int nSpeakerID);



/*===========================================================================*/
/* Load/Unload UserDict API */
#if !defined(VT_BASIC_DEFINE)
	/* Return Value */
	#define		VT_LOAD_USERDICT_SUCCESS					(1)
	#define		VT_LOAD_USERDICT_ERROR_INVALID_INDEX		(-1)
	#define		VT_LOAD_USERDICT_ERROR_INDEX_BUSY			(-2)
	#define		VT_LOAD_USERDICT_ERROR_LOAD_FAIL			(-3)
	#define		VT_LOAD_USERDICT_ERROR_UNKNOWN				(-4)

	#define		VT_UNLOAD_USERDICT_SUCCESS					(1)
	#define		VT_UNLOAD_USERDICT_ERROR_NULL_INDEX			(-1)
	#define		VT_UNLOAD_USERDICT_ERROR_INVALID_INDEX		(-2)
	#define		VT_UNLOAD_USERDICT_ERROR_UNKNOWN			(-3)
#endif

_DllMode(short) VT_LOAD_UserDict_JPN(int dictidx, char *filename);
_DllMode(short) VT_UNLOAD_UserDict_JPN(int dictidx);



/*===========================================================================*/
/* SOUND CARD API */
#if !defined(VT_BASIC_DEFINE)
	/* Return Value */
	#define		VT_PLAY_API_SUCCESS						(1)
	#define		VT_PLAY_API_ERROR_CREATE_THREAD			(-1)
	#define		VT_PLAY_API_ERROR_NULL_TEXT				(-2)
	#define		VT_PLAY_API_ERROR_EMPTY_TEXT			(-3)
	#define		VT_PLAY_API_ERROR_DB_NOT_LOADED			(-4)
	#define		VT_PLAY_API_ERROR_INITPLAY				(-5)
	#define		VT_PLAY_API_ERROR_UNKNOWN				(-6)
#endif

#if defined(WIN32)
	_DllMode(short) VT_PLAYTTS_JPN(HWND hcaller, UINT umsg, char *text_buff, int nSpeakerID, int pitch, int speed, int volume, int pause, int dictidx, int texttype);
	_DllMode(void)	VT_STOPTTS_JPN(void);
	_DllMode(void)	VT_RESTARTTTS_JPN(void);
	_DllMode(void)	VT_PAUSETTS_JPN(void);
#endif



/*===========================================================================*/
/* FILE WRITE API */
#if !defined(VT_BASIC_DEFINE)
	/* Return Value */
	#define		VT_FILE_API_SUCCESS						(1)
	#define		VT_FILE_API_ERROR_INVALID_FORMAT		(-1)
	#define		VT_FILE_API_ERROR_CREATE_THREAD			(-2)
	#define		VT_FILE_API_ERROR_NULL_TEXT				(-3)
	#define		VT_FILE_API_ERROR_EMPTY_TEXT			(-4)
	#define		VT_FILE_API_ERROR_DB_NOT_LOADED			(-5)
	#define		VT_FILE_API_ERROR_OUT_FILE_OPEN			(-6)
	#define		VT_FILE_API_ERROR_UNKNOWN				(-7)

	/* Audio Format */
	enum {
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
#endif

_DllMode(short) VT_TextToFile_JPN(int fmt, char *tts_text, char *filename, int nSpeakerID, int pitch, int speed, int volume, int pause, int dictidx, int texttype);



/*===========================================================================*/
/* BUFFER I/O API */
#if !defined(VT_BASIC_DEFINE)
	/* Return Value */
	#define		VT_BUFFER_API_PROCESSING					(0)
	#define		VT_BUFFER_API_DONE							(1)
	#define		VT_BUFFER_API_ERROR_INVALID_FORMAT			(-1)
	#define		VT_BUFFER_API_ERROR_CREATE_THREAD			(-2)
	#define		VT_BUFFER_API_ERROR_NULL_TEXT				(-3)
	#define		VT_BUFFER_API_ERROR_EMPTY_TEXT				(-4)
	#define		VT_BUFFER_API_ERROR_NULL_BUFFER				(-5)
	#define		VT_BUFFER_API_ERROR_DB_NOT_LOADED			(-6)
	#define		VT_BUFFER_API_ERROR_THREAD_BUSY				(-7)
	#define		VT_BUFFER_API_ERROR_ABNORMAL_CONDITION		(-8)
	#define		VT_BUFFER_API_ERROR_UNKNOWN					(-9)

	/* Audio Format */
	enum {
		VT_BUFFER_API_FMT_S16PCM = VT_FILE_API_FMT_S16PCM,
		VT_BUFFER_API_FMT_ALAW   = VT_FILE_API_FMT_ALAW,
		VT_BUFFER_API_FMT_MULAW  = VT_FILE_API_FMT_MULAW,
		VT_BUFFER_API_FMT_DADPCM = VT_FILE_API_FMT_DADPCM,
	};
#endif

_DllMode(int) VT_TextToBuffer_JPN(int fmt, char *tts_text, char *output_buff, int *output_len, int flag, int nThreadID, int nSpeakerID, int pitch, int speed, int volume, int pause, int dictidx, int texttype);



/*===========================================================================*/
/* CONFIGURE API */
_DllMode(void) VT_SetPitchSpeedVolumePause_JPN(int pitch, int speed, int volume, int pause, int nSpeakerID);
_DllMode(void) VT_SetCommaPause_JPN(int pause, int nSpeakerID);



/*===========================================================================
SYNOPSIS
	int VT_GetTTSInfo_JPN(int request, char *licensefile, void *value, int valuesize);

PARAMETERS
	request
		VT_BUILD_DATE			 (char*): library build date
		VT_VERIFY_CODE         	 (int *): verification result(licensefile is required)
		VT_MAX_CHANNEL         	 (int *): max no. of possible channels(licensefile is required)
		VT_DB_DIRECTORY        	 (char*): default root DB fold name
		VT_LOAD_SUCCESS_CODE   	 (int *): return value, when db loading is success
		VT_MAX_SPEAKER         	 (int *): max no. of speaker ( >= 0 )
		VT_DEF_SPEAKER         	 (int *): default speaker id ( >= 0 && < max no. of speaker )
		VT_CODEPAGE            	 (int *): supported ansi codepage (WIN32 only)
		VT_DB_ACCESS_MODE      	 (int *): file or ram i/o ? (file:0, ram:1)
		VT_FIXED_POINT_SUPPORT 	 (int *): fixed point simulated or not? (float:0, fixed:1)
		VT_SAMPLING_FREQUENCY  	 (int *): current sampling frequency (8000, 11025, 16000 )
		VT_MAX_PITCH_RATE      	 (int *): max value of pitch rate (%)
		VT_DEF_PITCH_RATE      	 (int *): default value of pitch rate (%)
		VT_MIN_PITCH_RATE      	 (int *): min value of pitch rate (%)
		VT_MAX_SPEED_RATE      	 (int *): max value of speed rate (%)
		VT_DEF_SPEED_RATE      	 (int *): default value of speed rate (%)
		VT_MIN_SPEED_RATE      	 (int *): min value of speed rate (%)
		VT_MAX_VOLUME          	 (int *): max value of volume (%)
		VT_DEF_VOLUME          	 (int *): default value of volume (%)
		VT_MIN_VOLUME          	 (int *): min value of volume (%)
		VT_MAX_SENT_PAUSE		 (int *): max value of sentence pause (msec)
		VT_DEF_SENT_PAUSE		 (int *): default value of sentence pause (msec)
		VT_MIN_SENT_PAUSE		 (int *): min value of sentence pause (msec)
		VT_DB_BUILD_DATE		 (char*): embedded db build date (for embedded engine only)
		VT_MAX_COMMA_PAUSE		 (int *): max value of comma pause (msec)
		VT_DEF_COMMA_PAUSE		 (int *): default value of comma pause (msec)
		VT_MIN_COMMA_PAUSE		 (int *): min value of comma pause (msec)

	licensefile
		if NULL, use default licensefile.

	value
		VT_DB_DIRECTORY and VT_BUILD_DATE requests are (char *), and any other request is (int *)

	valuesize
		maximum length of value in characters

RETURN VALUE
	On success, zero(VT_INFO_SUCCESS) is returned.
	On error, the return value depends on the operation:
		VT_INFO_ERROR_NOT_SUPPORTED_REQUEST	(1)
		VT_INFO_ERROR_INVALID_REQUEST		(2)
		VT_INFO_ERROR_NULL_VALUE			(3)
		VT_INFO_ERROR_SHORT_LENGTH_VALUE	(4)
		VT_INFO_ERROR_UNKNOWN				(5)
===========================================================================*/

#if !defined(VT_BASIC_DEFINE)
	/* Return Value */
	#define	VT_INFO_SUCCESS						(0)
	#define	VT_INFO_ERROR_NOT_SUPPORTED_REQUEST	(1)
	#define	VT_INFO_ERROR_INVALID_REQUEST		(2)
	#define VT_INFO_ERROR_NULL_VALUE			(3)
	#define	VT_INFO_ERROR_SHORT_LENGTH_VALUE	(4)
	#define	VT_INFO_ERROR_UNKNOWN				(5)

	/* Request */
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
#endif

_DllMode(int) VT_GetTTSInfo_JPN(int request, char *licensefile, void *value, int valuesize);


#if !defined(VT_BASIC_DEFINE)
	#define VT_BASIC_DEFINE
#endif

#if defined(__cplusplus)
	}
#endif

#endif /* VT_JPN_H */
