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

	hr = connectDevice();

	return hr;
}

bool KTMKinectWrapper::streamFromFile(char* fPath){
	if(isConnected())
		disconnectDevice();

	cv::namedWindow("Playback");

	inVideo.open(fPath);
	double t = inVideo.get(CV_CAP_PROP_FOURCC);

	streamingFromFile = inVideo.isOpened();

	return streamingFromFile;
}

bool KTMKinectWrapper::setOutFile(char* fileName){
	/* Prompt the user for a choice of codec */
	return setOutFile(fileName, NULL);
}

bool KTMKinectWrapper::setOutFile(char* fileName, char* c){
#ifdef KTM_USE_OPEN_CV
	/* If no codec is specified prompt the user for a codec */
	if(NULL != c)
		outVideo.open(fileName, CV_FOURCC(c[0],c[1],c[2],c[3]), 25, cv::Size(OUT_FRAME_WIDTH, OUT_FRAME_HEIGHT), true);
	else
		outVideo.open(fileName, -1, 30, cv::Size(OUT_FRAME_WIDTH, OUT_FRAME_HEIGHT), true);
	return outVideo.isOpened();
#endif
#ifdef KTM_USE_BOOST
	boostOutFile = new boost::iostreams::basic_file_sink<char>(fileName, std::ios_base::binary);
	return NULL != boostOutFile;
#endif
#ifdef KTM_USE_STD_FILE
	pFile = fopen(fileName, "wb");
	return NULL != pFile;
#endif
}

void KTMKinectWrapper::record(bool r){
#ifdef KTM_USE_OPEN_CV
	if(r)
		cv::namedWindow("Recording - Combined");
	else
		cv::destroyWindow("Recording - Combined");
#endif
	recordEnable = r;
}

bool KTMKinectWrapper::releaseOutFile(){
#ifdef KTM_USE_OPEN_CV
	if(outVideo.isOpened())
		outVideo.release();
#endif
#ifdef KTM_USE_BOOST
	if(NULL != boostOutFile){
		boostOutFile->flush();
		boostOutFile->close();
		boostOutFile = NULL;
	}
#endif
#ifdef KTM_USE_STD_FILE
	if(NULL != pFile){
		fclose(pFile);
		pFile = NULL;
	}
#endif
	return true;
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

USHORT* KTMKinectWrapper::nextFrame(){
	USHORT* dataDepth = NULL;
	char* RGBdata = NULL;

	if(streamingFromFile){
		dataDepth = this->getDepthFromFileStream();
	}else{
		RGBdata = this->getColorFromKinectStream();
		dataDepth = this->getDepthFromKinectStream();
	}

#ifdef KTM_USE_OPEN_CV	
	cv::Mat matCombined = cv::Mat::zeros(RGBA_FRAME_HEIGHT, RGBA_FRAME_WIDTH, CV_8UC4);
	cv::Mat matRGBA = cv::Mat::zeros(RGBA_FRAME_HEIGHT, RGBA_FRAME_WIDTH, CV_8UC4);
	if(NULL != RGBdata){
		matRGBA.data = (unsigned char*)RGBdata;
	}
	matCombined = matRGBA.clone();

	cv::Mat matDepth = cv::Mat::zeros(DEPTH_FRAME_HEIGHT / 2, DEPTH_FRAME_WIDTH, CV_8UC4);
	if(NULL != dataDepth){
		int dataSize = iFrameHeight * iFrameWidth * sizeof(USHORT);
		memcpy(matDepth.data, dataDepth, dataSize);
	}
	matCombined.push_back(matDepth);

	if(outVideo.isOpened() && !streamingFromFile && recordEnable){
		cv::imshow("Recording - Combined", matCombined);
		outVideo.write(matCombined);
	}
#endif
#ifdef KTM_USE_BOOST
	if(NULL != boostOutFile){
		if(boostOutFile->is_open() || !streamingFromFile && recordEnable){
			//if(NULL != RGBdata)
			//	boostOutFile->write(RGBdata, RGBA_FRAME_HEIGHT * RGBA_FRAME_WIDTH * OUT_FRAME_CHANNELS);
			//else
			//	boostOutFile->write(mRGBDataHeap, RGBA_FRAME_HEIGHT * RGBA_FRAME_WIDTH * OUT_FRAME_CHANNELS);

			if(NULL != dataDepth)
				boostOutFile->write((char*)dataDepth, DEPTH_FRAME_HEIGHT * DEPTH_FRAME_WIDTH * sizeof(USHORT));
			else
				boostOutFile->write((char*)mDataHeap, DEPTH_FRAME_HEIGHT * DEPTH_FRAME_WIDTH * sizeof(USHORT));

			boostOutFile->flush();
		}
	}
#endif
#ifdef KTM_USE_STD_FILE
	if(NULL != pFile){
		if(!streamingFromFile && recordEnable){
			if(NULL != RGBdata)
				fwrite(RGBdata, sizeof(char), RGBA_FRAME_HEIGHT * RGBA_FRAME_WIDTH * OUT_FRAME_CHANNELS, pFile);
			else
				fwrite(mRGBDataHeap, sizeof(char), RGBA_FRAME_HEIGHT * RGBA_FRAME_WIDTH * OUT_FRAME_CHANNELS, pFile);

			if(NULL != dataDepth)
				fwrite(dataDepth, sizeof(USHORT), DEPTH_FRAME_HEIGHT * DEPTH_FRAME_WIDTH, pFile);
			else
				fwrite(mDataHeap, sizeof(USHORT), DEPTH_FRAME_HEIGHT * DEPTH_FRAME_WIDTH, pFile);
		}
	}
#endif

	return dataDepth;
}

USHORT* KTMKinectWrapper::getDepthFromFileStream(){
	USHORT* frameData = mDataHeap;

	cv::Mat m(OUT_FRAME_HEIGHT, OUT_FRAME_WIDTH, CV_8UC4);
	int mdataSize = m.dataend - m.datastart;

	int t1= m.type();

	bool b = inVideo.set(CV_CAP_PROP_FORMAT, 1);

	inVideo.grab();
	if(!inVideo.retrieve(m, 3)){
		inVideo.set(CV_CAP_PROP_POS_FRAMES, 0);
		inVideo.grab();
		inVideo.retrieve(m, 3);
	}
	
	cv::imshow("Playback", m);

	int frameSize = iFrameWidth * iFrameHeight * sizeof(USHORT);
	unsigned char* cData = m.data;

	memcpy(frameData, cData, frameSize);
	return frameData;
}

USHORT* KTMKinectWrapper::getDepthFromKinectStream(){
	/* Check if the next depth frame resulst has been signalled, and if */
	/* not, return NULL (Don't block)                                   */
	if (WAIT_OBJECT_0 != WaitForSingleObject(hNextDepthFrameEvent, 0))
        return NULL;

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
	/* Check if the next depth frame resulst has been signalled, and if */
	/* not, return NULL (Don't block)                                   */
	if (WAIT_OBJECT_0 != WaitForSingleObject(hNextColorFrameEvent, 0))
        return NULL;

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