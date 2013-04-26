#pragma once

#include <pthread.h>
#include <queue>
#include <iostream>
#include <sstream>
#include <boost\iostreams\device\file.hpp>
#include <boost\iostreams\categories.hpp>
#include <boost\iostreams\concepts.hpp>
//#include "Util.hpp"
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

		boost::iostreams::basic_file_sink<char>* outFile;

		int framesWritten;
		bool writeProcessActive;
		pthread_t writeProcessThread;

		void* writeProcess(void* threadArgs);
		static void* writeProcessStarter(void* context);
		char* currentFilePath;
	public:
		bool init();
		bool setOutFile(char* filePath);
		bool flush();
		bool releaseOutFile();
		bool write(char* data, long size);
		bool writeBlank(long size);
		bool startWriteProcess();
		bool isWriting();
	};
};