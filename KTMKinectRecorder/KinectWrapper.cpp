#include "KinectWrapper.hpp"

KTM::KinectWrapper::KinectWrapper() :
	pKinectSensor(NULL),
	streamingFromFile(false),
	hNextDepthFrameEvent(NULL),
	hNextColorFrameEvent(NULL),
	recordEnable(false),
	outFileReady(false),
	outFileName(NULL),
	recordStartTime(0)
{
	mRGBDataHeap = NULL;
	mDepthDataHeap = NULL;
	setRGBRecordingResolution(NUI_IMAGE_RESOLUTION_640x480);
	setDepthRecordingResolution(NUI_IMAGE_RESOLUTION_640x480);
	fileWriter.init();
}

KTM::KinectWrapper::~KinectWrapper(){
	disconnectDevice();

	if(NULL != mDepthDataHeap){
		delete[] mDepthDataHeap;
		mDepthDataHeap = NULL;
	}

	if(NULL != mRGBDataHeap){
		delete[] mRGBDataHeap;
		mRGBDataHeap = NULL;
	}
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

	hasDepth = false;
	hasRGB = false;

	char info[9] = "";
	inFile.read(info, 5);
	info[5] = '\0';
	if(strcmp(info, "DEPTH") == 0){
		hasDepth = true;
		inFile.read(info, 8);
		info[8] = '\0';
		if(strcmp(info, "640X480 ") == 0)
			setDepthRecordingResolution(NUI_IMAGE_RESOLUTION_640x480);
		else if(strcmp(info, "320X240 ") == 0)
			setDepthRecordingResolution(NUI_IMAGE_RESOLUTION_320x240);
		else if(strcmp(info, "80X60   ") == 0)
			setDepthRecordingResolution(NUI_IMAGE_RESOLUTION_80x60);
		else
			return false;
	}else{
		inFile.seekg(-5, std::ios_base::cur);
	}

	inFile.read(info, 5);
	info[5] = '\0';
	if(strcmp(info, "RGB  ") == 0){
		hasRGB = true;
		inFile.read(info, 8);
		info[8] = '\0';
		if(strcmp(info, "1280X960") == 0)
			setRGBRecordingResolution(NUI_IMAGE_RESOLUTION_1280x960);
		else if(strcmp(info, "640X480 ") == 0)
			setRGBRecordingResolution(NUI_IMAGE_RESOLUTION_640x480);
		else
			return false;
	}else{
		inFile.seekg(-5, std::ios_base::cur);
	}

	playbackStartTime = timeGetTime();
	return streamingFromFile;
}

void KTM::KinectWrapper::skipFileInfo(){
	char info[9] = "";
	inFile.read(info, 5);
	info[5] = '\0';
	if(strcmp(info, "DEPTH") == 0){
		hasDepth = true;
		inFile.seekg(8, std::ios_base::cur);
	}else{
		inFile.seekg(-5, std::ios_base::cur);
	}

	inFile.read(info, 5);
	info[5] = '\0';
	if(strcmp(info, "RGB  ") == 0){
		inFile.seekg(8, std::ios_base::cur);
	}else{
		inFile.seekg(-5, std::ios_base::cur);
	}
}

bool KTM::KinectWrapper::setOutFile(char* fileName){
	/* Prompt the user for a choice of codec */
	return setOutFile(fileName, NULL);
}

bool KTM::KinectWrapper::setOutFile(char* fileName, char* c){
	outFileReady = fileWriter.setOutFile(fileName);
	int fileLen = strlen(fileName) + 1;
	outFileName = new char[fileLen];
	memcpy(outFileName, fileName, fileLen); 
	return outFileReady;
}

bool KTM::KinectWrapper::hasOutFile(){
	return outFileReady;
}

void KTM::KinectWrapper::record(bool r){
	recordEnable = r;
	if(NULL == outFileName)
		return;

	if(recordEnable){
		if(!fileWriter.isWriting()){
			fileWriter.setOutFile(outFileName);
		}

		recordStartTime = timeGetTime();
		if(hasDepth){
			fileWriter.write("DEPTH", 5);
			if(depthResolutionCode == NUI_IMAGE_RESOLUTION_640x480)
				fileWriter.write("640X480 ", 8);
			if(depthResolutionCode == NUI_IMAGE_RESOLUTION_320x240)
				fileWriter.write("320X240 ", 8);
			if(depthResolutionCode == NUI_IMAGE_RESOLUTION_80x60)
				fileWriter.write("80X60   ", 8);
		}
		if(hasRGB){
			fileWriter.write("RGB  ", 5);
			if(RGBResolutionCode == NUI_IMAGE_RESOLUTION_1280x960)
				fileWriter.write("1280X960", 8);
			if(RGBResolutionCode == NUI_IMAGE_RESOLUTION_640x480)
				fileWriter.write("640X480 ", 8);
		}
	}else{
		fileWriter.flush();
		recordStartTime = 0;
	}
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
	if(isConnected())
		disconnectDevice();

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
                RGBResolutionCode,
                0,
                NUI_IMAGE_STREAM_FRAME_LIMIT_MAXIMUM,
                hNextColorFrameEvent,
                &hColorStreamHandle
			);

			hr = pKinectSensor->NuiImageStreamOpen(
				NUI_IMAGE_TYPE_DEPTH,
				depthResolutionCode,
                0,
                NUI_IMAGE_STREAM_FRAME_LIMIT_MAXIMUM,
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

	if (NULL != hNextColorFrameEvent)
		if(hNextColorFrameEvent != INVALID_HANDLE_VALUE)
			CloseHandle(hNextColorFrameEvent);
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

void KTM::KinectWrapper::tiltUp(){
	long angle = 0;
	NuiCameraElevationGetAngle(&angle);
	if(angle < NUI_CAMERA_ELEVATION_MAXIMUM)
		angle += 10;
		if(angle > NUI_CAMERA_ELEVATION_MAXIMUM)
			angle = NUI_CAMERA_ELEVATION_MAXIMUM;
		NuiCameraElevationSetAngle(angle + 1);
}

void KTM::KinectWrapper::tiltDown(){
	long angle = 0;
	NuiCameraElevationGetAngle(&angle);
	if(angle > NUI_CAMERA_ELEVATION_MINIMUM)
		angle -= 10;
		if(angle < NUI_CAMERA_ELEVATION_MINIMUM)
			angle = NUI_CAMERA_ELEVATION_MINIMUM;
		NuiCameraElevationSetAngle(angle - 1);
}

void KTM::KinectWrapper::nextFrame(USHORT* &outDepthData, char* &outRGBData){
	USHORT* depthData = NULL;
	char* RGBdata = NULL;

	if(streamingFromFile){
		
		long frameTime = 0;
		if(hasDepth)
			depthData = this->getDepthFromFileStream(frameTime);
		if(hasRGB)
			RGBdata = this->getRGBAFromFileStream(frameTime);

		long currentTime = timeGetTime();
		long timeSincePlaybackStart = currentTime - playbackStartTime;
		if(timeSincePlaybackStart < frameTime)
			Sleep(frameTime - timeSincePlaybackStart);

		if(inFile.eof() || inFile.fail()){
			inFile.clear();
			inFile.seekg(0, std::ios_base::beg);
			skipFileInfo();
			playbackStartTime = timeGetTime();
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

		if(NULL != depthData && hasDepth){
			fileWriter.write("DEPTH", 5 * sizeof(char));
			fileWriter.write((char*)new long(timeSinceStart), sizeof(long));
			fileWriter.write((char*)depthData, depthFrameWidth * depthFrameHeight * sizeof(USHORT));
		}

		if(NULL != RGBdata && hasRGB){
			fileWriter.write("RGB  ", 5);
			fileWriter.write((char*)new long(timeSinceStart), sizeof(long));
			fileWriter.write(RGBdata, RGBFrameWidth * RGBFrameHeight * OUT_FRAME_CHANNELS * sizeof(unsigned char));
		}
	}

	outDepthData = NULL;
	outRGBData = NULL;

	if(hasDepth)
		outDepthData = depthData;
	if(hasRGB)
		outRGBData = RGBdata;
}

USHORT* KTM::KinectWrapper::getDepthFromFileStream(long &frameTime){
	USHORT* frameData = mDepthDataHeap;

	char* type = new char[6];
	inFile.read(type, 5);
	if(NULL != type){
		type[5] = '\0';
		if(strcmp(type, "DEPTH") != 0){
			inFile.seekg(-5, 0);
			delete type;
			return NULL;
		}
		delete type;
	}else{
		throw "No type info for frame!";
	}

	long* t = new long[1];
	inFile.read((char*)t, sizeof(long));
	if(t != NULL){
		frameTime = t[0];
		delete t;
	}else{
		throw "No time code for frame!";
	}

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
	if(NULL != type){
		type[5] = '\0';
		if(strcmp(type, "RGB  ") != 0){
			inFile.seekg(-5, 0);
			delete type;
			return NULL;
		}
		delete type;
	}else{
		throw "No type info for frame!";
	}

	long* t = new long[1];
	inFile.read((char*)t, sizeof(long));
	if(t != NULL){
		frameTime = t[0];
		delete t;
	}else{
		throw "No time code for frame!";
	}

	char* frameData = mRGBDataHeap;
	int frameSize = RGBFrameHeight * RGBFrameWidth * OUT_FRAME_CHANNELS * sizeof(unsigned char);
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
        const char* pBufferEnd = pBufferRun + (RGBFrameWidth * RGBFrameHeight * 4);
		if(OUT_FRAME_CHANNELS == 4){
			memcpy(mFrameData, pBufferRun, pBufferEnd - pBufferRun);
		}else{
			int s = pBufferEnd - pBufferRun;
			for(int i = 0, j = 0; i < s; i++){
				mFrameData[j++] = pBufferRun[i++];
				mFrameData[j++] = pBufferRun[i++];
				mFrameData[j++] = pBufferRun[i++];
			}
		}
    }

    /* Unlock the rectangle and release the frame */
    pTexture->UnlockRect(0);
    pKinectSensor->NuiImageStreamReleaseFrame(hColorStreamHandle, &imageFrame);

	return mFrameData;
}

bool KTM::KinectWrapper::setDepthRecordingResolution(NUI_IMAGE_RESOLUTION NUIAPICode){
	if(	NUIAPICode != NUI_IMAGE_RESOLUTION_640x480 && 
		NUIAPICode != NUI_IMAGE_RESOLUTION_320x240 && 
		NUIAPICode != NUI_IMAGE_RESOLUTION_80x60)
		return false;

	depthResolutionCode = NUIAPICode;

	switch(depthResolutionCode){
	case NUI_IMAGE_RESOLUTION_640x480:
		depthFrameWidth = 640;
		depthFrameHeight = 480;
		break;
	case NUI_IMAGE_RESOLUTION_320x240:
		depthFrameWidth = 320;
		depthFrameHeight = 240;
		break;
	case NUI_IMAGE_RESOLUTION_80x60:
		depthFrameWidth = 80;
		depthFrameHeight = 60;
		break;
	}

	if(NULL != mDepthDataHeap){
		delete[] mDepthDataHeap;
		mDepthDataHeap = NULL;
	}
	mDepthDataHeap = new USHORT[depthFrameHeight*depthFrameWidth];

	if(isConnected()){
		pKinectSensor->NuiImageStreamOpen(
			NUI_IMAGE_TYPE_DEPTH,
			depthResolutionCode,
            0,
            NUI_IMAGE_STREAM_FRAME_LIMIT_MAXIMUM,
            hNextDepthFrameEvent,
            &hDepthStreamHandle
		);
	}

	return true;
}

bool KTM::KinectWrapper::setRGBRecordingResolution(NUI_IMAGE_RESOLUTION NUIAPICode){
	if(NUIAPICode != NUI_IMAGE_RESOLUTION_1280x960 && NUIAPICode != NUI_IMAGE_RESOLUTION_640x480)
		return false;

	RGBResolutionCode = NUIAPICode;

	switch(RGBResolutionCode){
	case NUI_IMAGE_RESOLUTION_1280x960:
		RGBFrameWidth = 1280;
		RGBFrameHeight = 960;
		break;
	case NUI_IMAGE_RESOLUTION_640x480:
		RGBFrameWidth = 640;
		RGBFrameHeight = 480;
		break;
	}

	if(NULL != mRGBDataHeap){
		delete[] mRGBDataHeap;
		mRGBDataHeap = NULL;
	}
	mRGBDataHeap = new char[RGBFrameHeight * RGBFrameWidth * OUT_FRAME_CHANNELS];

	if(isConnected()){
		pKinectSensor->NuiImageStreamOpen(
			NUI_IMAGE_TYPE_COLOR,
            RGBResolutionCode,
            0,
            NUI_IMAGE_STREAM_FRAME_LIMIT_MAXIMUM,
            hNextColorFrameEvent,
            &hColorStreamHandle
		);
	}

	return true;
}

NUI_IMAGE_RESOLUTION KTM::KinectWrapper::getRGBResolutionCode(){
	return RGBResolutionCode;
}

NUI_IMAGE_RESOLUTION KTM::KinectWrapper::getDepthResolutionCode(){
	return depthResolutionCode;
}

void KTM::KinectWrapper::getDepthResolution(int &width, int &height){
	width = depthFrameWidth;
	height = depthFrameHeight;
}

void KTM::KinectWrapper::getRGBResolution(int &width, int &height){
	width = RGBFrameWidth;
	height = RGBFrameHeight;
}

void KTM::KinectWrapper::setRecordRGB(bool b){
	hasRGB = b;
}

void KTM::KinectWrapper::setRecordDepth(bool b){
	hasDepth = b;
}

bool KTM::KinectWrapper::getRecordRGB(){
	return hasRGB;
}

bool KTM::KinectWrapper::getRecordDepth(){
	return hasDepth;
}