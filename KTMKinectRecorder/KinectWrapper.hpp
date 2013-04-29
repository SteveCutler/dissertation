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
		char* outFileName;
		bool outFileReady;
		bool recordEnable;
		bool streamingFromFile;
		USHORT* mDepthDataHeap;
		char* mRGBDataHeap;
		long recordStartTime;
		long playbackStartTime;
		long playbackLastTime;
		long timeSinceStart;

		bool hasDepth;
		NUI_IMAGE_RESOLUTION depthResolutionCode;
		int depthFrameWidth;
		int depthFrameHeight;
		
		bool hasRGB;
		NUI_IMAGE_RESOLUTION RGBResolutionCode;
		int RGBFrameHeight;
		int RGBFrameWidth;

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
		void skipFileInfo();

		void tiltUp();
		void tiltDown();

		/* TODO: Remove! Replaced by setters and getters for resolution */
		int getFrameWidth(){ return depthFrameWidth; };
		int getFrameHeight(){ return depthFrameHeight; };

		/* Setters and getters for resolutions */
		bool setRGBRecordingResolution(NUI_IMAGE_RESOLUTION NUIAPICode);
		bool setDepthRecordingResolution(NUI_IMAGE_RESOLUTION NUIAPICode);
		NUI_IMAGE_RESOLUTION getRGBResolutionCode();
		NUI_IMAGE_RESOLUTION getDepthResolutionCode();
		void getRGBResolution(int &width, int &height);
		void getDepthResolution(int &width, int &height);

		/* Setters and getters for recording different streams */
		void setRecordRGB(bool b);
		void setRecordDepth(bool b);
		bool getRecordRGB();
		bool getRecordDepth();
	};
};
#endif