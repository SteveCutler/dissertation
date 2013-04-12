#include "ThreadedFileWriter.hpp"

bool KTM::ThreadedFileWriter::init(){
	dataQueueMutex = PTHREAD_MUTEX_INITIALIZER;
	outFileMutex = PTHREAD_MUTEX_INITIALIZER;
	return true;
}

bool KTM::ThreadedFileWriter::write(char* data, long size){
	KTM::DataToWrite entry;
	entry.data = data;
//	entry.data = new char[size];
//	memcpy(entry.data, data, size);
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

	return (bool)pthread_create(&writeProcessThread, &attr, &KTM::ThreadedFileWriter::writeProcessStarter, this);
}

void* KTM::ThreadedFileWriter::writeProcessStarter(void* context){
	return ((KTM::ThreadedFileWriter*)context)->writeProcess(NULL);
}

#ifdef KTM_USE_CV
bool KTM::ThreadedFileWriter::setOutFile(char* filePath){
	pthread_mutex_lock(&outFileMutex);
	outFile.open(filePath, CV_FOURCC('D','I','B',' '), 25, cv::Size(OUT_FRAME_WIDTH, OUT_FRAME_HEIGHT), true);
	pthread_mutex_unlock(&outFileMutex);
	return outFile.isOpened() && startWriteProcess();
}

bool KTM::ThreadedFileWriter::releaseOutFile(){
	pthread_mutex_lock(&outFileMutex);
	if(cvOutFile.isOpened())
		cvOutFile.release();
	pthread_mutex_unlock(&outFileMutex);
	return !outFile.isOpened();
}

void* KTM::ThreadedFileWriter::writeProcess(void* threadArgs){
	writeProcessActive = true;
	
	while(writeProcessActive){
		pthread_mutex_lock(&dataQueueMutex);
		if(dataQueue.empty()){
			pthread_mutex_unlock(&dataQueueMutex);
			continue;
		}

		char* data = dataQueue.front();
		dataQueue.pop();
		pthread_mutex_unlock(&dataQueueMutex);

		cv::Mat imageMat = cv::Mat::zeros(OUT_FRAME_HEIGHT, OUT_FRAME_WIDTH, CV_8UC4);
		pthread_mutex_lock(&outFileMutex);
		cvOutFile.write(imageMat);
		pthread_mutex_unlock(&outFileMutex);
	}

	return NULL;
}
#endif

#ifdef KTM_USE_BOOST
bool KTM::ThreadedFileWriter::setOutFile(char* filePath){
	pthread_mutex_lock(&outFileMutex);
	outFile = new boost::iostreams::basic_file_sink<char>(filePath, std::ios_base::binary);
	pthread_mutex_unlock(&outFileMutex);
	startWriteProcess();
	return NULL != outFile;
}

bool KTM::ThreadedFileWriter::releaseOutFile(){
	bool queueEmpty = false;
	while(!queueEmpty){
		pthread_mutex_lock(&dataQueueMutex);
		int s = dataQueue.size();
		queueEmpty = dataQueue.empty();
		pthread_mutex_unlock(&dataQueueMutex);
	}

	writeProcessActive = false;

	pthread_mutex_lock(&outFileMutex);
	if(NULL != outFile){
		outFile->close();
		outFile = NULL;
	}
	pthread_mutex_unlock(&outFileMutex);

	pthread_join(writeProcessThread, NULL);
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
#endif