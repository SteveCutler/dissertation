#pragma once

#include <Windows.h>

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