#pragma once

#include <pthread.h>
#include <opencv\cv.h>

class KTMThreadedFileWriter{
private:
public:
	static bool setOutFile(char* filePath);
	static bool write(char* data);
}