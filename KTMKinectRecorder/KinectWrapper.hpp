#ifndef KTM_KINECT_WRAPPER_H
#define KTM_KINECT_WRAPPER_H

#include <Windows.h>
#include "Util.hpp"
#include "NuiApi.h"
#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <stdio.h>
#include <opencv\cv.h>
#include <opencv\highgui.h>
#include <boost\iostreams\device\file.hpp>
#include <boost\iostreams\categories.hpp>
#include <boost\iostreams\concepts.hpp>

#include "ThreadedFileWriter.hpp"
#include "NotificationInterface.hpp"

namespace KTM{
	#define FILE_FRAME_SEPERATOR ""

	#define DEPTH_FRAME_WIDTH 640
	#define DEPTH_FRAME_HEIGHT 480

	class KinectWrapper{
	private:
		bool outputToFile;
		bool recordEnable;
		bool streamingFromFile;
		char* filePath;
		USHORT* mDataHeap;
		char* mRGBDataHeap;
		unsigned char* mOutDataHeap;

		int iFrameWidth;
		int iFrameHeight;
		int iBytesPerPixel;

		HANDLE hDepthStreamHandle;
		HANDLE hNextDepthFrameEvent;
		HANDLE hColorStreamHandle;
		HANDLE hNextColorFrameEvent;
		INuiSensor* pKinectSensor;

		cv::VideoCapture inVideo;
		std::ifstream inFile;
		int framesInFile;
		int framesReadFromFile;

		KTM::ThreadedFileWriter fileWriter;
	
		IplImage* imgHeap;
		cv::Mat* matHeap;

		boost::iostreams::basic_file_sink<char>* boostOutFile;
		FILE* pFile;

		/* Private Functions */
		USHORT* getDepthFromFileStream();
		char* getRGBAFromFileStream();
		USHORT* getDepthFromKinectStream();
		bool getDepthAndColorFromKinectStream(USHORT* depth, char* color);
		char* getColorFromKinectStream();

		/* Different File Output Methods */
		int writeMethod;
		bool writeCV(char*, USHORT*);
		bool writeSTDFile(char*, USHORT*);
		bool writeBoost(char*, USHORT*);
	public:
		KinectWrapper();
		~KinectWrapper();
	
		HRESULT streamFromKinect();
		HRESULT connectDevice();
		void disconnectDevice();
		bool isConnected();
		void setNearMode(bool);
		void nextFrame(USHORT* &outDepthData, char* &outRGBData);
		bool setOutFile(char*);
		void record(bool);
		bool setOutFile(char*, char*);
		bool releaseOutFile();
		bool releaseInFile();
		bool streamFromFile(char*);
		int getFrameWidth(){ return iFrameWidth; };
		int getFrameHeight(){ return iFrameHeight; };

		/* Different Writing Methods */
		void setWriteMethod(int);
	};
};
#endif