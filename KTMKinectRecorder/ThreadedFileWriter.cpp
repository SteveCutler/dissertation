#include "ThreadedFileWriter.hpp"

bool KTM::ThreadedFileWriter::init(){
	dataQueueMutex = PTHREAD_MUTEX_INITIALIZER;
	outFileMutex = PTHREAD_MUTEX_INITIALIZER;
	writeProcessActive = false;
	return true;
}

bool KTM::ThreadedFileWriter::write(char* data, long size){
	KTM::DataToWrite entry;
	entry.data = data;
	entry.size = size;
	pthread_mutex_lock(&dataQueueMutex);
	dataQueue.push(entry);
	pthread_mutex_unlock(&dataQueueMutex);
	return true;
}

bool KTM::ThreadedFileWriter::writeBlank(long size){
	KTM::DataToWrite entry;
	char* d = new char[size];
	entry.data = d;
	entry.size = size;
	pthread_mutex_lock(&dataQueueMutex);
	dataQueue.push(entry);
	pthread_mutex_unlock(&dataQueueMutex);
	return true;
}

bool KTM::ThreadedFileWriter::startWriteProcess(){
	pthread_attr_t attr;
	pthread_attr_init(&attr);
	pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_JOINABLE);
	if(writeProcessActive)
		return false;
	return pthread_create(&writeProcessThread, &attr, &KTM::ThreadedFileWriter::writeProcessStarter, this) == 0;
}

void* KTM::ThreadedFileWriter::writeProcessStarter(void* context){
	return ((KTM::ThreadedFileWriter*)context)->writeProcess(NULL);
}

bool KTM::ThreadedFileWriter::setOutFile(char* filePath){
	pthread_mutex_lock(&outFileMutex);

	outFile = new boost::iostreams::basic_file_sink<char>(filePath, std::ios_base::binary);
	if(NULL == outFile){
		pthread_mutex_unlock(&outFileMutex);
		return false;
	}
	pthread_mutex_unlock(&outFileMutex);
	if(!writeProcessActive)
		return startWriteProcess();
	else
		return true;
}

bool KTM::ThreadedFileWriter::releaseOutFile(){
	bool queueEmpty = false;
	KTM::NotificationInterface::setMessage(L"Waiting for file output to finish...");
	
	while(!queueEmpty){
		pthread_mutex_lock(&dataQueueMutex);
		int s = dataQueue.size();
		std::wstringstream ss;
		ss << "Waiting for file output to finish: " << s;
		KTM::NotificationInterface::setMessage((wchar_t*)ss.str().c_str());
		queueEmpty = dataQueue.empty();
		pthread_mutex_unlock(&dataQueueMutex);
		Sleep(500);
	}

	writeProcessActive = false;

	KTM::NotificationInterface::setMessage(L"Closing file...");
	pthread_mutex_lock(&outFileMutex);
	if(NULL != outFile){
		outFile->close();
		outFile = NULL;
	}
	pthread_mutex_unlock(&outFileMutex);

	pthread_join(writeProcessThread, NULL);

	KTM::NotificationInterface::setMessage(L"File output complete!");
	return NULL == outFile;
}

void* KTM::ThreadedFileWriter::writeProcess(void* threadArgs){
	writeProcessActive = true;
	framesWritten = 0;
	while(writeProcessActive){
		pthread_mutex_lock(&dataQueueMutex);
		if(dataQueue.empty()){
			pthread_mutex_unlock(&dataQueueMutex);
			continue;
		}
		int size = dataQueue.size();
		KTM::DataToWrite entry = dataQueue.front();
		char* data = entry.data;
		pthread_mutex_unlock(&dataQueueMutex);

		pthread_mutex_lock(&outFileMutex);
		if(NULL != outFile){
			if(outFile->is_open()){
				outFile->write(data, entry.size);
				outFile->flush();
			}
		}

//		delete data;

		pthread_mutex_unlock(&outFileMutex);

		pthread_mutex_lock(&dataQueueMutex);
		dataQueue.pop();
		framesWritten++;
		pthread_mutex_unlock(&dataQueueMutex);
	}

	return NULL;
}