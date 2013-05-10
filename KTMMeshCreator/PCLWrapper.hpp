#pragma once

#define NOMINMAX
#include <Windows.h>
#include <pcl/visualization/cloud_viewer.h>
#include <boost/make_shared.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/point_representation.h>

#include <pcl/io/pcd_io.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/filter.h>

#include <pcl/features/normal_3d.h>

#include <pcl/common/geometry.h>

#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/registration/transforms.h>

#include <pcl/visualization/pcl_visualizer.h>
#include <KinectWrapper.hpp>
#include <NuiApi.h>

#include <pthread.h>

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloud;
typedef pcl::PointNormal PointNormalT;
typedef pcl::PointCloud<PointNormalT> PointCloudWithNormals;

namespace KTM{
	typedef struct _depthArrayToPointCloudArgs{
		int numThreads;
		int threadIndex;
		unsigned short* depthData;
		PointCloud::Ptr transformationMatrix;
		PointCloud::Ptr outCloud;
	} depthArrayToPointCloudArgs;
	void* t_depthArrayToPointCloud(void* threadArgs);

	class PCLWrapper{
	private:
		pcl::PointCloud<pcl::PointXYZ>::Ptr mergedCloud;
		PointCloud::Ptr prevCloud;
		pcl::PointCloud<pcl::PointXYZ>::Ptr depthTransformationMatrix;
		NUI_IMAGE_RESOLUTION depthTransformationMatrixResolution;
		pcl::visualization::CloudViewer* cloudViewer;
		Eigen::Matrix4f GlobalTransform;
		bool firstRun;
	public:
		PCLWrapper();
		~PCLWrapper();
		
		void updateTransformationMatrix(NUI_IMAGE_RESOLUTION res);
		bool addToCloud(unsigned short* depthData, int depthDataWidth, int depthDataHeight);
		bool addToCloud(unsigned short* depthData, int depthDataWidth, int depthDataHeight, char* RGBData, int RGBDataWidth, int RGBDataHeight);
	};
};