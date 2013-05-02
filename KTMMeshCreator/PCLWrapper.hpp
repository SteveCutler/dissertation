#pragma once

#pragma comment( lib, "pcl_visualization_debug.lib" )

#include <pcl/visualization/cloud_viewer.h>
#include <pcl/point_types.h>

namespace KTM{
	class PCLWrapper{
	private:
		pcl::PointCloud<pcl::PointXYZ> mergedCloud;
		boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>>* p_mergedCloud;
		pcl::visualization::CloudViewer* cloudViewer;
	public:
		PCLWrapper();
		~PCLWrapper();

		bool addToCloud(unsigned short* depthData, int depthDataWidth, int depthDataHeight);
		bool addToCloud(unsigned short* depthData, int depthDataWidth, int depthDataHeight, char* RGBData, int RGBDataWidth, int RGBDataHeight);
	};
};