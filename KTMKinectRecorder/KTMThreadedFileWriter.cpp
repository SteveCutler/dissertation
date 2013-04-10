#include "KTMThreadedFileWriter.hpp"

bool KTM::ThreadedFileWriter::init(){
	dataQueueMutex = PTHREAD_MUTEX_INITIALIZER;
	outFileMutex = PTHREAD_MUTEX_INITIALIZER;
	return startWriteProcess();
}

bool KTM::ThreadedFileWriter::setOutFile(char* filePath){
	pthread_mutex_lock(&outFileMutex);
	outFile.open(filePath, CV_FOURCC('D','I','B',' '), 25, cv::Size(OUT_FRAME_WIDTH, OUT_FRAME_HEIGHT), true);
	pthread_mutex_unlock(&outFileMutex);
	return outFile.isOpened();
}

bool KTM::ThreadedFileWriter::write(char* data){
	pthread_mutex_lock(&outFileMutex);
	dataQueue.push(data);
	pthread_mutex_unlock(&outFileMutex);
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
		outFile.write(imageMat);
		pthread_mutex_unlock(&outFileMutex);
	}

	return NULL;
}