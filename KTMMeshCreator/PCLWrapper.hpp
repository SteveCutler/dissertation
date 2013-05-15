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

#include <Eigen/Eigen>
#include <Eigen/SVD>
#include <Eigen/StdVector>

#include <pcl/visualization/pcl_visualizer.h>
#include <KinectWrapper.hpp>
#include <NuiApi.h>

#include <pthread.h>
#include "PCThreading.hpp"

typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloud;
typedef pcl::PointNormal PointNormalT;
typedef pcl::PointCloud<PointNormalT> PointCloudWithNormals;

#define KTM_PCLWRAPPER_COST_THRESHOLD 0.5f
#define KTM_PCLWRAPPER_MAX_ITER 15
#define KTM_PCLWRAPPER_DIST_THRESHOLD 10.0f
#define KTM_PCLWRAPPER_ICP_NORM_THRESHOLD 6.0f

namespace KTM{
	class PCLWrapper{
	private:
		PointCloud::Ptr mergedCloud;
		PointCloud::Ptr prevCloud;
		Eigen::Vector4f* prevPoints;
		Eigen::Vector4f* prevNormals;
		unsigned short* prevDepthData;
		pcl::PointCloud<pcl::PointXYZ>::Ptr depthTransformationMatrix;
		NUI_IMAGE_RESOLUTION depthTransformationMatrixResolution;
		pcl::visualization::CloudViewer* cloudViewer;
		Eigen::Matrix4f GlobalTransform;
		Eigen::Matrix4f prevTransform;
		bool firstRun;
		float* TAN;
	public:
		PCLWrapper();
		~PCLWrapper();
		
		void updateTransformationMatrix(NUI_IMAGE_RESOLUTION res);
		bool addToCloud(unsigned short* depthData, int depthDataWidth, int depthDataHeight);
		bool addToCloud(unsigned short* depthData, int depthDataWidth, int depthDataHeight, char* RGBData, int RGBDataWidth, int RGBDataHeight);
		void ICP(unsigned short* depthData,int dataSize,Eigen::Vector4f* depthCloud,Eigen::Vector4f* depthCloudNormals,unsigned short* prevDepthData ,Eigen::Vector4f* prevDepthCloud,Eigen::Vector4f* prevDepthCloudNormals, Eigen::Matrix4f& guessTransform, Eigen::Matrix4f* estimatedTransform, float costThreshHold = KTM_PCLWRAPPER_COST_THRESHOLD, int maxIterations = KTM_PCLWRAPPER_MAX_ITER, float distanceThreshold = KTM_PCLWRAPPER_DIST_THRESHOLD,float normalThreshold = KTM_PCLWRAPPER_ICP_NORM_THRESHOLD);
	};
};