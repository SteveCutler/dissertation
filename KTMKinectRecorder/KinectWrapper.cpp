#include "KinectWrapper.hpp"

KTM::KinectWrapper::KinectWrapper() :
	pKinectSensor(NULL),
	depthFrameWidth(640),
	depthFrameHeight(480),
	streamingFromFile(false),
	hNextDepthFrameEvent(NULL),
	hNextColorFrameEvent(NULL),
	recordEnable(false),
	outFileReady(false),
	recordStartTime(0)
{
	mDepthDataHeap = new USHORT[depthFrameHeight*depthFrameWidth];
	mRGBDataHeap = new char[640*480*4];
	fileWriter.init();
}

KTM::KinectWrapper::~KinectWrapper(){
	disconnectDevice();

    delete[] mDepthDataHeap;
}

HRESULT KTM::KinectWrapper::streamFromKinect(){
	HRESULT hr;

	streamingFromFile = false;
	inFile.close();

	hr = connectDevice();

	return hr;
}

bool KTM::KinectWrapper::streamFromFile(char* fPath){
	if(inFile.is_open())
		releaseInFile();

	inFile.open(fPath, std::ios::in | std::ios::binary);
	
	streamingFromFile = inFile.is_open();

	inFile.clear();
	inFile.seekg(0, std::ios_base::beg);
	playbackStartTime = timeGetTime();
	return streamingFromFile;
}

bool KTM::KinectWrapper::setOutFile(char* fileName){
	/* Prompt the user for a choice of codec */
	return setOutFile(fileName, NULL);
}

bool KTM::KinectWrapper::setOutFile(char* fileName, char* c){
	outFileReady = fileWriter.setOutFile(fileName);
	return outFileReady;
}

bool KTM::KinectWrapper::hasOutFile(){
	return outFileReady;
}

void KTM::KinectWrapper::record(bool r){
	recordEnable = r;
	if(recordEnable)
		recordStartTime = timeGetTime();
	else
		recordStartTime = 0;
}

bool KTM::KinectWrapper::releaseOutFile(){
	fileWriter.releaseOutFile();
	return true;
}

bool KTM::KinectWrapper::releaseInFile(){
	if(inFile.is_open())
		inFile.close();
	return !inFile.is_open();
}

HRESULT KTM::KinectWrapper::connectDevice(){
    INuiSensor * pTmpKinectSensor;
    HRESULT hr;

    int iSensorCount = 0;
    hr = NuiGetSensorCount(&iSensorCount);
    if (FAILED(hr))
        return hr;

    // Look at each Kinect sensor
    for (int i = 0; i < iSensorCount; ++i){
        // Create the sensor so we can check status, if we can't create it, move on to the next
        hr = NuiCreateSensorByIndex(i, &pTmpKinectSensor);
        if (FAILED(hr))
            continue;

        // Get the status of the sensor, and if connected, then we can initialize it
        hr = pTmpKinectSensor->NuiStatus();
        if (S_OK == hr){
            pKinectSensor = pTmpKinectSensor;
            break;
        }

        // This sensor wasn't OK, so release it since we're not using it
        pTmpKinectSensor->Release();
    }

    if (NULL != pKinectSensor){
        // Initialize the Kinect and specify that we'll be using depth
		hr = pKinectSensor->NuiInitialize(NUI_INITIALIZE_FLAG_USES_COLOR | NUI_INITIALIZE_FLAG_USES_DEPTH);
        if (SUCCEEDED(hr)){
            // Create an event that will be signaled when depth data is available
			hNextDepthFrameEvent = CreateEvent(NULL, TRUE, FALSE, NULL);
			hNextColorFrameEvent = CreateEvent(NULL, TRUE, FALSE, NULL);

            // Open a depth image stream to receive depth frames
			hr = pKinectSensor->NuiImageStreamOpen(
				NUI_IMAGE_TYPE_COLOR,
                NUI_IMAGE_RESOLUTION_640x480,
                0,
                NUI_IMAGE_STREAM_FRAME_LIMIT_MAXIMUM,
                hNextColorFrameEvent,
                &hColorStreamHandle
			);

			hr = pKinectSensor->NuiImageStreamOpen(
				NUI_IMAGE_TYPE_DEPTH,
                NUI_IMAGE_RESOLUTION_640x480,
                0,
                2,
                hNextDepthFrameEvent,
                &hDepthStreamHandle
			);
        }
    }

    if (NULL == pKinectSensor || FAILED(hr))
        return E_FAIL;

    return hr;
}

void KTM::KinectWrapper::disconnectDevice(){
	/* Shutdown the Kinect, if active */
	if (pKinectSensor){
		pKinectSensor->NuiShutdown();
		pKinectSensor = NULL;
	}

	/* Release the handle on the next frame event, if valid */
	if (NULL != hNextDepthFrameEvent)
		if(hNextDepthFrameEvent != INVALID_HANDLE_VALUE)
			CloseHandle(hNextDepthFrameEvent);
}

bool KTM::KinectWrapper::isConnected(){
	if(pKinectSensor)
		return true;
	return false;
}

void KTM::KinectWrapper::setNearMode(bool t){
	if (NULL != pKinectSensor)
		pKinectSensor->NuiImageStreamSetImageFrameFlags(hDepthStreamHandle, t ? NUI_IMAGE_STREAM_FLAG_ENABLE_NEAR_MODE : 0);
}

void KTM::KinectWrapper::nextFrame(USHORT* &outDepthData, char* &outRGBData){
	USHORT* depthData = NULL;
	char* RGBdata = NULL;

	if(streamingFromFile){
		
		long frameTime = 0;
		depthData = this->getDepthFromFileStream(frameTime);
		RGBdata = this->getRGBAFromFileStream(frameTime);

		long currentTime = timeGetTime();
		long timeSincePlaybackStart = currentTime - playbackStartTime;
		if(timeSincePlaybackStart < frameTime)
			Sleep(frameTime - timeSincePlaybackStart);

		if(inFile.eof()){
			inFile.clear();
			inFile.seekg(0, std::ios_base::beg);
			playbackLastTime = timeGetTime();
		}
	}else{
		if(NULL == pKinectSensor)
			return;

		WaitForSingleObject(hNextDepthFrameEvent, 1000);
		RGBdata = this->getColorFromKinectStream();

		WaitForSingleObject(hNextColorFrameEvent, 1000);
		depthData = this->getDepthFromKinectStream();
	}

	if(recordEnable){
		long currentTime = timeGetTime();
		timeSinceStart = currentTime - recordStartTime;

		if(NULL != depthData){
			fileWriter.write("DEPTH", 5 * sizeof(char));
			fileWriter.write((char*)new long(timeSinceStart), sizeof(long));
			fileWriter.write((char*)depthData, DEPTH_FRAME_HEIGHT * DEPTH_FRAME_WIDTH * sizeof(USHORT));
		}

		if(NULL != RGBdata){
			fileWriter.write("RGB  ", 5);
			fileWriter.write((char*)new long(timeSinceStart), sizeof(long));
			fileWriter.write(RGBdata, RGBA_FRAME_HEIGHT * RGBA_FRAME_WIDTH * 4 * sizeof(unsigned char));
		}
	}

	outDepthData = depthData;
	outRGBData = RGBdata;
}

USHORT* KTM::KinectWrapper::getDepthFromFileStream(long &frameTime){
	USHORT* frameData = mDepthDataHeap;

	char* type = new char[6];
	inFile.read(type, 5);
	type[5] = '\0';
	if(strcmp(type, "DEPTH") != 0){
		inFile.seekg(-5, 0);
		delete type;
		return NULL;
	}
	delete type;

	long* t = new long[1];
	inFile.read((char*)t, sizeof(long));
	frameTime = t[0];
	delete t;

	int frameSize = depthFrameWidth * depthFrameHeight * sizeof(unsigned short);
	if(!inFile.is_open())
		return NULL;

	USHORT* cData = mDepthDataHeap;

	inFile.read((char*)cData, frameSize);

	memcpy(frameData, cData, frameSize);
	return frameData;
}

char* KTM::KinectWrapper::getRGBAFromFileStream(long &frameTime){
	char* type = new char[6];
	inFile.read(type, 5);
	type[5] = '\0';
	if(strcmp(type, "RGB  ") != 0){
		inFile.seekg(-5, 0);
		delete type;
		return NULL;
	}
	delete type;

	long* t = new long[1];
	inFile.read((char*)t, sizeof(long));
	frameTime = t[0];
	delete t;

	char* frameData = mRGBDataHeap;
	int frameSize = RGBA_FRAME_HEIGHT * RGBA_FRAME_WIDTH * 4 * sizeof(unsigned char);
	if(!inFile.is_open())
		return NULL;

	char* cData = mRGBDataHeap;

	
	inFile.read((char*)cData, frameSize);

	memcpy(frameData, cData, frameSize);
	return frameData;
}

USHORT* KTM::KinectWrapper::getDepthFromKinectStream(){
    HRESULT hr;

	/* Get the next frame from the Kinect stream */
    NUI_IMAGE_FRAME imageFrame;
    hr = pKinectSensor->NuiImageStreamGetNextFrame(hDepthStreamHandle, 0, &imageFrame);

	/* Quit if the frame couldn't be recieved */
    if (FAILED(hr))
        return NULL;

	/* Get the frame as a texture so we can get the frame rectangle */
	/* and lock the handle on the frame so it's not overwritten by  */
	/* the Kinect while we're using it                              */
    INuiFrameTexture* pTexture = imageFrame.pFrameTexture;
    NUI_LOCKED_RECT lockedRect;
    pTexture->LockRect(0, &lockedRect, NULL, 0);

    /* Make sure we have valid data */
    if (lockedRect.Pitch != 0){
		/* Use the assigned data heap to avoid memory leaks and avoid */
		/* needlessly assigning new memory                            */
        USHORT* mFrameData = mDepthDataHeap;

		/* Pointer to the top left corner of rectangle, and pointer */
		/* to the frame end                                         */
        const USHORT* pBufferRun = (const unsigned short*)lockedRect.pBits;
        const USHORT* pBufferEnd = pBufferRun + (depthFrameWidth * depthFrameHeight);

        while ( pBufferRun < pBufferEnd ){
            /* discard the portion of the depth that contains only the player index */
            USHORT depth = NuiDepthPixelToDepth(*pBufferRun);

            /* Convert the returned depth to a char, discarding the most significant */
			/* bits in favor of the least significant                                */
            USHORT intensity = depth;

            /* Save the depth data */
            *(mFrameData++) = intensity;

            // Increment our index into the Kinect's depth buffer
            ++pBufferRun;
        }
    }

    /* Unlock the rectangle and release the frame */
    pTexture->UnlockRect(0);
    pKinectSensor->NuiImageStreamReleaseFrame(hDepthStreamHandle, &imageFrame);

	return mDepthDataHeap;
}

char* KTM::KinectWrapper::getColorFromKinectStream(){
    HRESULT hr;

	/* Get the next frame from the Kinect stream */
    NUI_IMAGE_FRAME imageFrame;
    hr = pKinectSensor->NuiImageStreamGetNextFrame(hColorStreamHandle, 0, &imageFrame);

	/* Quit if the frame couldn't be recieved */
    if (FAILED(hr))
        return NULL;

	/* Get the frame as a texture so we can get the frame rectangle */
	/* and lock the handle on the frame so it's not overwritten by  */
	/* the Kinect while we're using it                              */
    INuiFrameTexture* pTexture = imageFrame.pFrameTexture;
    NUI_LOCKED_RECT lockedRect;
    pTexture->LockRect(0, &lockedRect, NULL, 0);

	/* Use the assigned data heap to avoid memory leaks and avoid */
	/* needlessly assigning new memory                            */
	char* mFrameData = mRGBDataHeap;

    /* Make sure we have valid data */
    if (lockedRect.Pitch != 0){
		

		/* Pointer to the top left corner of rectangle, and pointer */
		/* to the frame end                                         */
        const char* pBufferRun = (const char*)lockedRect.pBits;
        const char* pBufferEnd = pBufferRun + (depthFrameWidth * depthFrameHeight * 4);

		memcpy(mFrameData, pBufferRun, pBufferEnd - pBufferRun);
    }

    /* Unlock the rectangle and release the frame */
    pTexture->UnlockRect(0);
    pKinectSensor->NuiImageStreamReleaseFrame(hColorStreamHandle, &imageFrame);

	return mFrameData;
}