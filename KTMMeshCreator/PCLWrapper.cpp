#include "PCLWrapper.hpp"

KTM::PCLWrapper::PCLWrapper(){
	cloudViewer = new pcl::visualization::CloudViewer("Cloud Viewer");
	p_mergedCloud = new boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>>(&mergedCloud);
}

KTM::PCLWrapper::~PCLWrapper(){
}

bool KTM::PCLWrapper::addToCloud(unsigned short* depthData, int depthDataWidth, int depthDataHeight){
	return addToCloud(depthData, depthDataWidth, depthDataHeight, NULL, 0, 0);
}

bool KTM::PCLWrapper::addToCloud(unsigned short* depthData, int depthDataWidth, int depthDataHeight, char* RGBData, int RGBDataWidth, int RGBDataHeight){
	if(NULL == depthData)
		return false;
	
	mergedCloud.clear();

	if(NULL != depthData){
		int index = 0;
		for(int dCol = 0; dCol < depthDataHeight; dCol++){
			for(int dRow = 0; dRow < depthDataWidth; dRow++){
				unsigned short d = depthData[index++];
				if(d > 0)
					mergedCloud.push_back(pcl::PointXYZ(dRow, dCol, d));
			}
		}
	}

	cloudViewer->showCloud(*p_mergedCloud, "Cloud");
	return true;
}