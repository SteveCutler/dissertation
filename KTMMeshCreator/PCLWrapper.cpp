#include "PCLWrapper.hpp"

KTM::PCLWrapper::PCLWrapper(){
	mergedCloud = PointCloud::Ptr(new PointCloud);
	prevCloud = PointCloud::Ptr(new PointCloud);
	cloudViewer = new pcl::visualization::CloudViewer("Cloud Viewer");
	firstRun = true;
	GlobalTransform = Eigen::Matrix4f::Identity();
	depthTransformationMatrixResolution = NUI_IMAGE_RESOLUTION_INVALID;
	TAN = NULL;
	prevPoints = NULL;
	prevNormals = NULL;
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

	if(TAN != NULL)
		delete[] TAN;

	TAN = new float(height);

//	KTM::PCThreading::createTANTable(height, 43.0f, TAN);
}

bool KTM::PCLWrapper::addToCloud(unsigned short* depthData, int depthDataWidth, int depthDataHeight){
	return addToCloud(depthData, depthDataWidth, depthDataHeight, NULL, 0, 0);
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

	PointCloud::Ptr depthCloud(new PointCloud);
	depthCloud->resize(depthDataWidth * depthDataHeight);

	if(depthRes != depthTransformationMatrixResolution){
		updateTransformationMatrix(depthRes);
		prevCloud->resize(depthDataWidth * depthDataHeight);
	}

	
	Eigen::Vector4f* points = new Eigen::Vector4f[depthDataWidth * depthDataHeight];
	Eigen::Vector4f* normals = new Eigen::Vector4f[depthDataWidth * depthDataHeight];
	KTM::PCThreading::depthArrayToVectorPC(depthData, depthDataWidth, depthDataHeight, depthTransformationMatrix, points);
	KTM::PCThreading::generateNormalsFromVectors(points,depthDataWidth,depthDataHeight,normals);

	//-----------------------------
	// TESTING
	//-----------------------------

	//Eigen::Matrix4f testTransform;
	//Eigen::Matrix4f testEstimatedTransform;
	//testTransform << 
	//	0.996, 0.086, -0.087, 0,
	//	-0.087, 0.996, 0, 0.,
	//	0.087, 0.008, 0.996, 0,
	//	0, 0, 0, 1;

	////testTransform << 
	////	1,0,0, 0.1,
	////	0,1, 0, 0.08,
	////	0,0,1, 0.06,
	////	0, 0, 0, 1;

	//Eigen::Vector4f* testPoints = new Eigen::Vector4f[depthDataWidth * depthDataHeight];
	//Eigen::Vector4f* testNormals = new Eigen::Vector4f[depthDataWidth * depthDataHeight];

	//for(int i = 0; i < depthDataWidth * depthDataHeight; i++){
	//	testPoints[i] = testTransform * points[i];
	//	testNormals[i] = testTransform * normals[i];
	//}

	//ICP(
	//	depthData,
	//	depthDataWidth * depthDataHeight,
	//	points,
	//	normals,
	//	depthData,
	//	testPoints,
	//	testNormals,
	//	GlobalTransform,
	//	&testEstimatedTransform
	//);

	//-----------------------------
	// \TESTING
	//-----------------------------

	//KTM::PCThreading::depthArrayToPointCloud(depthData, depthTransformationMatrix, depthCloud);

	if(firstRun){
		firstRun = false;
		prevTransform = GlobalTransform;
	}else{
		Eigen::Matrix4f estimatedTransform;
		ICP(
			depthData,
			depthDataWidth * depthDataHeight,
			points,
			normals,
			prevDepthData,
			prevPoints,
			prevNormals,
			prevTransform,
			&estimatedTransform
		);
		prevTransform = estimatedTransform;
		GlobalTransform = estimatedTransform.inverse() * GlobalTransform;
	}

	KTM::PCThreading::vectorPCToPCLPC(points, depthDataWidth * depthDataHeight, depthCloud);
	pcl::transformPointCloud(*depthCloud, *depthCloud, GlobalTransform);

	if(NULL != prevPoints)
		delete[] prevPoints;

	if(NULL != prevNormals)
		delete[] prevNormals;
	prevPoints = points;
	prevNormals = normals;
	prevDepthData = depthData;

	cloudViewer->showCloud(depthCloud, "Cloud");
	
	return true;
}

void KTM::PCLWrapper::ICP(
	unsigned short* depthData,
	int dataSize,
	Eigen::Vector4f* depthCloud,
	Eigen::Vector4f* depthCloudNormals,
	unsigned short* prevDepthData ,
	Eigen::Vector4f* prevDepthCloud,
	Eigen::Vector4f* prevDepthCloudNormals,
	Eigen::Matrix4f& guessTransform,
	Eigen::Matrix4f* estimatedTransform,
	float costThreshold,
	int maxIterations,
	float distanceThreshold,
	float normalThreshold
){
	*estimatedTransform = guessTransform;
	float sum = 10000;
	float prevSum = -1;
	int iteration = 0;

	while(sum > costThreshold && iteration++ < maxIterations){
		prevSum = sum;
		sum = 0;
		std::vector<const Eigen::Vector3f, Eigen::aligned_allocator<const Eigen::Vector3f>> x;
		std::vector<const Eigen::Vector3f, Eigen::aligned_allocator<const Eigen::Vector3f>> p;
		Eigen::Vector3f xCoM = Eigen::Vector3f::Zero();
		Eigen::Vector3f pCoM = Eigen::Vector3f::Zero();
		for(int i = 0; i < dataSize; i++){
			if(depthData[i] > 0 && prevDepthData[i] > 0){
				Eigen::Vector4f v = *estimatedTransform * depthCloud[i];
				Eigen::Vector4f n = *estimatedTransform * depthCloudNormals[i];
				
				Eigen::Vector4f prevV = prevDepthCloud[i];
				Eigen::Vector4f prevN = prevDepthCloudNormals[i];
				if((v - prevV).norm() < distanceThreshold && abs(n.dot(prevN)) < normalThreshold){
					float part = (v - prevV).norm();
					part = part * part;
					sum += part;
					Eigen::Vector3f v3f(v.x(), v.y(), v.z());
					Eigen::Vector3f prevV3f(prevV.x(), prevV.y(), prevV.z());
					x.push_back(prevV3f);
					p.push_back(v3f);
					xCoM += prevV3f;
					pCoM += v3f;
				}
			}
		}

		if(sum < costThreshold)
			break;

		xCoM = xCoM / x.size();
		pCoM = pCoM / p.size();

		Eigen::Matrix3f W = Eigen::Matrix3f::Zero();
		for(int i = 0; i < x.size(); i++){
			Eigen::Vector3f x_ = x[i] - xCoM;
			Eigen::Vector3f p_ = p[i] - pCoM;
			W = W + (x_ * p_.transpose());
		}
		
		Eigen::JacobiSVD<Eigen::Matrix3f> svd(W, Eigen::ComputeFullU | Eigen::ComputeFullV);
		bool u = svd.computeU();
		bool v = svd.computeV();

		Eigen::LLT<Eigen::Matrix3f> choleskyDecomp(W);
		choleskyDecomp.compute(W);
		
		Eigen::Matrix3f U = svd.matrixU();
		Eigen::Matrix3f V = svd.matrixV();

		Eigen::Matrix3f R = U * V.transpose();
		Eigen::Vector3f t =  xCoM - (R * pCoM);
		
		(*estimatedTransform)(0,0) = R(0,0);
		(*estimatedTransform)(0,1) = R(0,1);
		(*estimatedTransform)(0,2) = R(0,2);
		
		(*estimatedTransform)(1,0) = R(1,0);
		(*estimatedTransform)(1,1) = R(1,1);
		(*estimatedTransform)(1,2) = R(1,2);
		
		(*estimatedTransform)(2,0) = R(2,0);
		(*estimatedTransform)(2,1) = R(2,1);
		(*estimatedTransform)(2,2) = R(2,2);
		
		(*estimatedTransform)(0,3) = t(0);
		(*estimatedTransform)(1,3) = t(1);
		(*estimatedTransform)(2,3) = t(2);
		
		(*estimatedTransform)(3,3) = 1.0f;
	}
}