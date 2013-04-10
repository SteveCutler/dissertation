#pragma once

#include <pthread.h>
#include <opencv\cv.h>
#include <opencv\highgui.h>
#include <queue>

#define RGBA_FRAME_WIDTH 640
#define RGBA_FRAME_HEIGHT 480
#define OUT_FRAME_WIDTH 640
#define OUT_FRAME_HEIGHT 720
#define OUT_FRAME_CHANNELS 4

namespace KTM{
	class ThreadedFileWriter{
	private:
		typedef  void* (*Thread2Ptr)(void);
		typedef  void* (*PthreadPtr)(void*);

		pthread_mutex_t dataQueueMutex;
		pthread_mutex_t outFileMutex;
		std::queue<char*> dataQueue;
		cv::VideoWriter outFile;
		bool writeProcessActive;
		pthread_t writeProcessThread;

		void* writeProcess(void* threadArgs);
		static void* writeProcessStarter(void* context);
	public:
		bool init();
		bool setOutFile(char* filePath);
		bool write(char* data);
		bool startWriteProcess();
		
	};
};