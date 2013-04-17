#ifndef KTM_KINECT_WRAPPER_H
#define KTM_KINECT_WRAPPER_H


#include <windows.h>
#include <mmsystem.h>
#include "Util.hpp"
#include "NuiApi.h"
#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <stdio.h>
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
		bool outFileReady;
		bool recordEnable;
		bool streamingFromFile;
		USHORT* mDepthDataHeap;
		char* mRGBDataHeap;
		long recordStartTime;
		long playbackStartTime;
		long playbackLastTime;
		long timeSinceStart;

		int depthFrameWidth;
		int depthFrameHeight;

		HANDLE hDepthStreamHandle;
		HANDLE hNextDepthFrameEvent;
		HANDLE hColorStreamHandle;
		HANDLE hNextColorFrameEvent;
		INuiSensor* pKinectSensor;

		std::ifstream inFile;

		KTM::ThreadedFileWriter fileWriter;

		/* Private Functions */
		USHORT* getDepthFromFileStream(long &frameTime);
		char* getRGBAFromFileStream(long &frameTime);
		USHORT* getDepthFromKinectStream();
		bool getDepthAndColorFromKinectStream(USHORT* depth, char* color);
		char* getColorFromKinectStream();

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
		bool hasOutFile();
		void record(bool);
		bool setOutFile(char*, char*);
		bool releaseOutFile();
		bool releaseInFile();
		bool streamFromFile(char*);
		int getFrameWidth(){ return depthFrameWidth; };
		int getFrameHeight(){ return depthFrameHeight; };
	};
};
#endif