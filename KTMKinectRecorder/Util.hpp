#pragma once

#include <Windows.h>

#define RGBA_FRAME_WIDTH 640
#define RGBA_FRAME_HEIGHT 480
#define OUT_FRAME_WIDTH 640
#define OUT_FRAME_HEIGHT 480
#define OUT_FRAME_CHANNELS 3

static char* USHORTToChar(USHORT* data, UINT size){
	int diff = sizeof(USHORT) / sizeof(char);
	char* resultData = new char[size * diff];
	memcpy(resultData, data, size * diff);
	return resultData;
}

static USHORT* CharToUSHORT(char* data, UINT size){
	USHORT* resultData = new USHORT[size];
	memcpy(resultData, data, size);
	return resultData;
}

//
// Usage: SetThreadName (-1, "MainThread");
//
typedef struct tagTHREADNAME_INFO
{
   DWORD dwType; // must be 0x1000
   LPCSTR szName; // pointer to name (in user addr space)
   DWORD dwThreadID; // thread ID (-1=caller thread)
   DWORD dwFlags; // reserved for future use, must be zero
} THREADNAME_INFO;

static void SetThreadName( DWORD dwThreadID, LPCSTR szThreadName)
{
   THREADNAME_INFO info;
   info.dwType = 0x1000;
   info.szName = szThreadName;
   info.dwThreadID = dwThreadID;
   info.dwFlags = 0;

   __try
   {
      RaiseException( 0x406D1388, 0, sizeof(info)/sizeof(DWORD), (DWORD*)&info );
   }
__except(EXCEPTION_CONTINUE_EXECUTION)
{
}
}