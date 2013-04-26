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