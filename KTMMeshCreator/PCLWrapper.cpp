#include "PCLWrapper.hpp"

KTM::PCLWrapper::PCLWrapper(){
	mergedCloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
	cloudViewer = new pcl::visualization::CloudViewer("Cloud Viewer");
	firstRun = true;
	GlobalTransform = Eigen::Matrix4f::Identity();
	depthTransformationMatrixResolution = NUI_IMAGE_RESOLUTION_INVALID;
}

KTM::PCLWrapper::~PCLWrapper(){
}

void KTM::PCLWrapper::updateTransformationMatrix(NUI_IMAGE_RESOLUTION res){
	int width;
	int height;

	if(res == NUI_IMAGE_RESOLUTION_640x480){
		width = 640;
		height = 480;
	}else if(res == NUI_IMAGE_RESOLUTION_320x240){
		width = 320;
		height = 240;
	}else if(res == NUI_IMAGE_RESOLUTION_80x60){
		width = 80;
		height = 60;
	}else{
		return;
	}

	depthTransformationMatrix = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);

	for(int i = 0; i < height; i++){
		for(int j = 0; j < width; j++){
			Vector4 v = NuiTransformDepthImageToSkeleton(i, j, 1000, res);
			depthTransformationMatrix->push_back(pcl::PointXYZ(-v.x / 1000.0f, v.y / 1000.0f, v.z / 1000.0f));
		}
	}

	depthTransformationMatrixResolution = res;
}

bool KTM::PCLWrapper::addToCloud(unsigned short* depthData, int depthDataWidth, int depthDataHeight){
	return addToCloud(depthData, depthDataWidth, depthDataHeight, NULL, 0, 0);
}

void* KTM::t_depthArrayToPointCloud(void* threadArgs){
	depthArrayToPointCloudArgs* args = (depthArrayToPointCloudArgs*)threadArgs;
	int pointsPerThread = args->transformationMatrix->size() / args->numThreads;

	for(int i = args->threadIndex * pointsPerThread; i < (args->threadIndex + 1) * pointsPerThread && i < args->transformationMatrix->size(); i++){
		if(args->depthData[i] > 0){
			args->outCloud->points[i].x = args->transformationMatrix->points[i].x * (float)args->depthData[i];
			args->outCloud->points[i].y = args->transformationMatrix->points[i].y * (float)args->depthData[i];
			args->outCloud->points[i].z = args->transformationMatrix->points[i].z * (float)args->depthData[i];
		}
	}
	return NULL;
}

bool KTM::PCLWrapper::addToCloud(unsigned short* depthData, int depthDataWidth, int depthDataHeight, char* RGBData, int RGBDataWidth, int RGBDataHeight){
	if(NULL == depthData)
		return false;
	
	NUI_IMAGE_RESOLUTION depthRes;
	switch(depthDataWidth){
	case 640:
		depthRes = NUI_IMAGE_RESOLUTION_640x480;
		break;
	case 320:
		depthRes = NUI_IMAGE_RESOLUTION_320x240;
		break;
	case 80:
		depthRes = NUI_IMAGE_RESOLUTION_80x60;
		break;
	}

	if(depthRes != depthTransformationMatrixResolution)
		updateTransformationMatrix(depthRes);

	pcl::PointCloud<pcl::PointXYZ>::Ptr depthCloud (new pcl::PointCloud<pcl::PointXYZ>);
	depthCloud->resize(depthDataWidth * depthDataHeight);

	int numThreads = 8;
	pthread_t* threads = new pthread_t[numThreads];
	depthArrayToPointCloudArgs* args = new depthArrayToPointCloudArgs[numThreads];
	pthread_attr_t attr;
	pthread_attr_init(&attr);
	pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_JOINABLE);
	
	for(int i = 0; i < numThreads; i++){
		std::stringstream ss;
		ss << "DepthArrayToPointCloud Thread " << i;
		args[i].numThreads = numThreads;
		args[i].threadIndex = i;
		args[i].outCloud = depthCloud;
		args[i].depthData = depthData;
		args[i].transformationMatrix = depthTransformationMatrix;
		pthread_create(&threads[i], &attr, t_depthArrayToPointCloud, (void*)&args[i]);
	}

	for(int i = 0; i < numThreads; i++){
		pthread_join(threads[i], NULL);
	}

	delete[] args;

	if(firstRun){
		firstRun = false;
		prevCloud = depthCloud;
		mergedCloud = depthCloud;
	}else{
		pcl::IterativeClosestPoint<PointT,PointT> icp;
		icp.setInputCloud(prevCloud);
		icp.setInputTarget(depthCloud);
		icp.align(*depthCloud);
		pcl::transformPointCloud(*depthCloud, *depthCloud, GlobalTransform);
		GlobalTransform = icp.getFinalTransformation() * GlobalTransform;
		*mergedCloud = *mergedCloud + *depthCloud;
		prevCloud = depthCloud;
	}
	cloudViewer->showCloud(mergedCloud, "Cloud");
	
	return true;
}