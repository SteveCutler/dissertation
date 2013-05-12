#include "PCLWrapper.hpp"

KTM::PCLWrapper::PCLWrapper(){
	mergedCloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
	prevCloud = PointCloud::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
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

	pcl::PointCloud<pcl::PointXYZ>::Ptr depthCloud(new pcl::PointCloud<pcl::PointXYZ>);
	depthCloud->resize(depthDataWidth * depthDataHeight);

	if(depthRes != depthTransformationMatrixResolution){
		updateTransformationMatrix(depthRes);
		prevCloud->resize(depthDataWidth * depthDataHeight);
	}

	
	Eigen::Vector4f* points = new Eigen::Vector4f[depthDataWidth * depthDataHeight];
	Eigen::Vector4f* normals = new Eigen::Vector4f[depthDataWidth * depthDataHeight];
	KTM::PCThreading::depthArrayToVectorPC(depthData, depthDataWidth, depthDataHeight, depthTransformationMatrix, points);
	KTM::PCThreading::generateNormalsFromVectors(points,depthDataWidth,depthDataHeight,normals);

	Eigen::Vector4f p;
	for(int i = 0; i < depthDataWidth * depthDataHeight; i++){
		p = points[i];
	}
	//KTM::PCThreading::depthArrayToPointCloud(depthData, depthTransformationMatrix, depthCloud);

	if(firstRun){
		firstRun = false;
		//mergedCloud = depthCloud;	
	}else{
		//ICP(
		//	depthData,
		//	depthDataWidth * depthDataHeight,
		//	points,
		//	normals,
		//	prevDepthData,
		//	prevPoints,
		//	prevNormals,
		//	GlobalTransform
		//);
	}

	KTM::PCThreading::vectorPCToPCLPC(points, depthDataWidth * depthDataHeight, depthCloud);

	//*prevCloud = *depthCloud;
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
	Eigen::Matrix4f& estimatedTransform,
	float costThreshold,
	int maxIterations,
	float distanceThreshold,
	float normalThreshold
){
	float sum = 1000;
	int iteration = 0;

	while(sum > costThreshold && iteration++ < maxIterations){
		sum = 0;
		std::vector<const Eigen::Vector4f, Eigen::aligned_allocator<const Eigen::Vector4f>> x;
		std::vector<const Eigen::Vector4f, Eigen::aligned_allocator<const Eigen::Vector4f>> p;
		Eigen::Vector4f xCoM;
		Eigen::Vector4f pCoM;
		for(int i = 0; i < dataSize; i++){
			if(depthData[i] > 0 && prevDepthData[i] > 0){
				Eigen::Vector4f v = estimatedTransform * depthCloud[i];
				Eigen::Vector4f n = estimatedTransform * depthCloudNormals[i];
				
				Eigen::Vector4f prevV = prevDepthCloud[i];
				Eigen::Vector4f prevN = prevDepthCloudNormals[i];
				if((v - prevV).norm() < distanceThreshold && abs(n.dot(prevN)) < normalThreshold){
					float part = (v - prevV).dot(prevN);
					part = part * part;
					sum += part;
					x.push_back(prevV);
					p.push_back(v);
					xCoM += prevV;
					pCoM += v;
				}
			}
		}

		xCoM = xCoM / x.size();
		pCoM = pCoM / p.size();

		std::vector<const Eigen::Vector4f, Eigen::aligned_allocator<const Eigen::Vector4f>> x_;
		std::vector<const Eigen::Vector4f, Eigen::aligned_allocator<const Eigen::Vector4f>> p_;

		for(int i = 0; i < xCoM.size(); i++){
			x_.push_back(x[i] - xCoM);
			p_.push_back(p[i] - pCoM);
		}

		Eigen::Matrix4f W;

		for(int i = 0; i < x_.size(); i++){
			W = W + (x_[i] * p_[i].transpose());
		}
		
		Eigen::JacobiSVD<Eigen::Matrix4f> svd;
		svd.compute(W);
		Eigen::Matrix4f U = svd.matrixU();
		Eigen::Matrix4f V = svd.matrixV();

		Eigen::Matrix4f R = U * V.transpose();
		Eigen::Vector4f t = xCoM - R * pCoM;
		
		estimatedTransform(0,0) = R(0,0);
		estimatedTransform(0,1) = R(0,1);
		estimatedTransform(0,2) = R(0,2);
		
		estimatedTransform(1,0) = R(1,0);
		estimatedTransform(1,1) = R(1,1);
		estimatedTransform(1,2) = R(1,2);
		
		estimatedTransform(2,0) = R(2,0);
		estimatedTransform(2,1) = R(2,1);
		estimatedTransform(2,2) = R(2,2);
		
		estimatedTransform(3,0) = t(0);
		estimatedTransform(3,1) = t(1);
		estimatedTransform(3,2) = t(2);
		
		estimatedTransform(3,3) = 1.0f;
	}
}