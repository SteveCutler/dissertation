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

#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/registration/transforms.h>

#include <pcl/visualization/pcl_visualizer.h>
#include <KinectWrapper.hpp>
#include <NuiApi.h>

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloud;
typedef pcl::PointNormal PointNormalT;
typedef pcl::PointCloud<PointNormalT> PointCloudWithNormals;

namespace KTM{
	class PointAndCurvatureRepresentation : public pcl::PointRepresentation <PointNormalT>{
	  using pcl::PointRepresentation<PointNormalT>::nr_dimensions_;
	public:
	  PointAndCurvatureRepresentation ()
	  {
		// Define the number of dimensions
		nr_dimensions_ = 4;
	  }

	  // Override the copyToFloatArray method to define our feature vector
	  virtual void copyToFloatArray (const PointNormalT &p, float * out) const
	  {
		// < x, y, z, curvature >
		out[0] = p.x;
		out[1] = p.y;
		out[2] = p.z;
		out[3] = p.curvature;
	  }
	};

	class PCLWrapper{
	private:
		pcl::PointCloud<pcl::PointXYZ>::Ptr mergedCloud;
		pcl::PointCloud<pcl::PointXYZ>::Ptr depthTransformationMatrix;
		NUI_IMAGE_RESOLUTION depthTransformationMatrixResolution;
		pcl::visualization::CloudViewer* cloudViewer;
		Eigen::Matrix4f GlobalTransform;
		bool firstRun;

		void pairAlign (const PointCloud::Ptr cloud_src, const PointCloud::Ptr cloud_tgt, PointCloud::Ptr output, Eigen::Matrix4f &final_transform, bool downsample);
	public:
		PCLWrapper();
		~PCLWrapper();
		
		void updateTransformationMatrix(NUI_IMAGE_RESOLUTION res);
		bool addToCloud(unsigned short* depthData, int depthDataWidth, int depthDataHeight);
		bool addToCloud(unsigned short* depthData, int depthDataWidth, int depthDataHeight, char* RGBData, int RGBDataWidth, int RGBDataHeight);
	};
};