#include "KTMKinectWrapper.hpp"

KTMKinectWrapper::KTMKinectWrapper() :
	pKinectSensor(NULL),
	iFrameWidth(640),
	iFrameHeight(480),
	iBytesPerPixel(1),
	streamingFromFile(false),
	hNextDepthFrameEvent(NULL),
	hNextColorFrameEvent(NULL),
	recordEnable(false),
	boostOutFile(NULL),
	pFile(NULL)
{
	mDataHeap = new USHORT[iFrameHeight*iFrameWidth];
	mRGBDataHeap = new char[640*480*4];
	matHeap = new cv::Mat(640,480,CV_16U);
	imgHeap = cvCreateImage(cvSize(640,480),IPL_DEPTH_16U,1);
	fileWriter.init();
}

KTMKinectWrapper::~KTMKinectWrapper(){
	disconnectDevice();

    delete[] mDataHeap;
}

HRESULT KTMKinectWrapper::streamFromKinect(){
	HRESULT hr;
	
	cv::destroyWindow("Playback");

	streamingFromFile = false;
	inVideo.release();
	inFile.close();

	hr = connectDevice();

	return hr;
}

bool KTMKinectWrapper::streamFromFile(char* fPath){
	if(inFile.is_open())
		releaseInFile();

	inFile.open(fPath, std::ios::in | std::ios::binary);
	
	streamingFromFile = inFile.is_open();

	long start = inFile.tellg();
	inFile.seekg(0, std::ios_base::end);
	long end = inFile.tellg();

	long size = end - start;
	float frames = (float)size / (float)(iFrameHeight *iFrameWidth * sizeof(unsigned short));
	inFile.clear();
	inFile.seekg(0, std::ios_base::beg);
	return streamingFromFile;
}

bool KTMKinectWrapper::setOutFile(char* fileName){
	/* Prompt the user for a choice of codec */
	return setOutFile(fileName, NULL);
}

bool KTMKinectWrapper::setOutFile(char* fileName, char* c){
	return fileWriter.setOutFile(fileName);
}

void KTMKinectWrapper::record(bool r){
	recordEnable = r;
}

bool KTMKinectWrapper::releaseOutFile(){
	fileWriter.releaseOutFile();
	return true;
}

bool KTMKinectWrapper::releaseInFile(){
	if(inFile.is_open())
		inFile.close();
	return !inFile.is_open();
}

HRESULT KTMKinectWrapper::connectDevice(){
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

void KTMKinectWrapper::disconnectDevice(){
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

bool KTMKinectWrapper::isConnected(){
	if(pKinectSensor)
		return true;
	return false;
}

void KTMKinectWrapper::setNearMode(bool t){
	if (NULL != pKinectSensor)
		pKinectSensor->NuiImageStreamSetImageFrameFlags(hDepthStreamHandle, t ? NUI_IMAGE_STREAM_FLAG_ENABLE_NEAR_MODE : 0);
}

void KTMKinectWrapper::nextFrame(USHORT* &outDepthData, char* &outRGBData){
	USHORT* depthData = NULL;
	char* RGBdata = NULL;

	if(streamingFromFile){
		depthData = this->getDepthFromFileStream();
		RGBdata = this->getRGBAFromFileStream();
	}else{
		if(NULL == pKinectSensor)
			return;

		WaitForSingleObject(hNextDepthFrameEvent, 1000);
		RGBdata = this->getColorFromKinectStream();

		WaitForSingleObject(hNextColorFrameEvent, 1000);
		depthData = this->getDepthFromKinectStream();
	}

	if(recordEnable){
		if(NULL != depthData)
			fileWriter.write((char*)depthData, DEPTH_FRAME_HEIGHT * DEPTH_FRAME_WIDTH * sizeof(USHORT));
		else
			fileWriter.write((char*)mDataHeap, DEPTH_FRAME_HEIGHT * DEPTH_FRAME_WIDTH * sizeof(USHORT));

		if(NULL != RGBdata)
			fileWriter.write(RGBdata, RGBA_FRAME_HEIGHT * RGBA_FRAME_WIDTH * 4 * sizeof(unsigned char));
		else
			fileWriter.write(mRGBDataHeap, RGBA_FRAME_HEIGHT * RGBA_FRAME_WIDTH * 4 * sizeof(unsigned char));
	}

	outDepthData = depthData;
	outRGBData = RGBdata;
}

USHORT* KTMKinectWrapper::getDepthFromFileStream(){
	USHORT* frameData = mDataHeap;
	int frameSize = iFrameWidth * iFrameHeight * sizeof(unsigned short);
	if(!inFile.is_open())
		return NULL;

	USHORT* cData = mDataHeap;

	if(inFile.eof()){
		inFile.clear();
		inFile.seekg(0, std::ios_base::beg);
	}
	inFile.read((char*)cData, frameSize);

	memcpy(frameData, cData, frameSize);
	return frameData;
}

char* KTMKinectWrapper::getRGBAFromFileStream(){
	char* frameData = mRGBDataHeap;
	int frameSize = RGBA_FRAME_HEIGHT * RGBA_FRAME_WIDTH * 4 * sizeof(unsigned char);
	if(!inFile.is_open())
		return NULL;

	char* cData = mRGBDataHeap;

	if(inFile.eof()){
		inFile.clear();
		inFile.seekg(0, std::ios_base::beg);
	}
	inFile.read((char*)cData, frameSize);

	memcpy(frameData, cData, frameSize);
	return frameData;
}

USHORT* KTMKinectWrapper::getDepthFromKinectStream(){
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
        USHORT* mFrameData = mDataHeap;

		/* Pointer to the top left corner of rectangle, and pointer */
		/* to the frame end                                         */
        const USHORT* pBufferRun = (const unsigned short*)lockedRect.pBits;
        const USHORT* pBufferEnd = pBufferRun + (iFrameWidth * iFrameHeight);

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

	return mDataHeap;
}

char* KTMKinectWrapper::getColorFromKinectStream(){
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
        const char* pBufferEnd = pBufferRun + (iFrameWidth * iFrameHeight * 4);

		memcpy(mFrameData, pBufferRun, pBufferEnd - pBufferRun);
    }

    /* Unlock the rectangle and release the frame */
    pTexture->UnlockRect(0);
    pKinectSensor->NuiImageStreamReleaseFrame(hColorStreamHandle, &imageFrame);

	return mFrameData;
}