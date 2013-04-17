#pragma once

#include <pthread.h>
#include <opencv\cv.h>
#include <opencv\highgui.h>
#include <queue>
#include <iostream>
#include <sstream>
#include <boost\iostreams\device\file.hpp>
#include <boost\iostreams\categories.hpp>
#include <boost\iostreams\concepts.hpp>

#define RGBA_FRAME_WIDTH 640
#define RGBA_FRAME_HEIGHT 480
#define OUT_FRAME_WIDTH 640
#define OUT_FRAME_HEIGHT 480
#define OUT_FRAME_CHANNELS 4

//#define KTM_USE_CV
#define KTM_USE_BOOST
//#define KTM_USE_STD

#include "NotificationInterface.hpp"

namespace KTM{
	struct DataToWrite{
		char* data;
		long size;
	};

	class ThreadedFileWriter{
	private:
		typedef  void* (*Thread2Ptr)(void);
		typedef  void* (*PthreadPtr)(void*);

		pthread_mutex_t dataQueueMutex;
		pthread_mutex_t outFileMutex;
		std::queue<KTM::DataToWrite> dataQueue;

		/* File Writers */
#ifdef KTM_USE_CV
		cv::VideoWriter cvOutFile;
#endif
#ifdef KTM_USE_BOOST
		boost::iostreams::basic_file_sink<char>* outFile;
#endif
		int framesWritten;
		bool writeProcessActive;
		pthread_t writeProcessThread;

		void* writeProcess(void* threadArgs);
		static void* writeProcessStarter(void* context);
	public:
		bool init();
		bool setOutFile(char* filePath);
		bool releaseOutFile();
		bool write(char* data, long size);
		bool writeBlank(long size);
		bool startWriteProcess();
		bool isWriting();
	};
};