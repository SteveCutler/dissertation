#include "PCLWrapper.hpp"

KTM::PCLWrapper::PCLWrapper(){
	mergedCloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
	cloudViewer = new pcl::visualization::CloudViewer("Cloud Viewer");
	firstRun = true;
	GlobalTransform = Eigen::Matrix4f::Identity();
}

KTM::PCLWrapper::~PCLWrapper(){
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

	pcl::PointCloud<pcl::PointXYZ>::Ptr depthCloud (new pcl::PointCloud<pcl::PointXYZ>);
	depthCloud->resize(depthDataWidth * depthDataHeight);

	if(NULL != depthData){
		int index = 0;
		for(int dCol = 0; dCol < depthDataHeight; dCol++){
			for(int dRow = 0; dRow < depthDataWidth; dRow++){
				unsigned short d = depthData[index];
				if(d > 0){
					NUI_DEPTH_IMAGE_POINT depthPoint;
					depthPoint.depth = d;
					depthPoint.x = dCol;
					depthPoint.y = dRow;
					Vector4 p = NuiTransformDepthImageToSkeleton(dRow, dCol, d, depthRes);
					depthCloud->points[index].x = p.x;
					depthCloud->points[index].y = p.y;
					depthCloud->points[index].z = p.z;
				}
				index++;
			}
		}
	}

	if(firstRun){
		mergedCloud = depthCloud;
		firstRun = false;
	}else{
		Eigen::Matrix4f pairTransform;
		pcl::PointCloud<pcl::PointXYZ>::Ptr tmp(new pcl::PointCloud<pcl::PointXYZ>);
		pairAlign(depthCloud, mergedCloud, tmp, pairTransform, true);
		pcl::transformPointCloud(*tmp, *mergedCloud, GlobalTransform);
		GlobalTransform = pairTransform * GlobalTransform;
	}

	cloudViewer->showCloud(mergedCloud, "Cloud");
	return true;
}

void KTM::PCLWrapper::pairAlign (const PointCloud::Ptr cloud_src, const PointCloud::Ptr cloud_tgt, PointCloud::Ptr output, Eigen::Matrix4f &final_transform, bool downsample)
{
  //
  // Downsample for consistency and speed
  // \note enable this for large datasets
  PointCloud::Ptr src (new PointCloud);
  PointCloud::Ptr tgt (new PointCloud);
  pcl::VoxelGrid<PointT> grid;
  if (downsample)
  {
    grid.setLeafSize (0.05, 0.05, 0.05);
    grid.setInputCloud (cloud_src);
    grid.filter (*src);

    grid.setInputCloud (cloud_tgt);
    grid.filter (*tgt);
  }
  else
  {
    src = cloud_src;
    tgt = cloud_tgt;
  }


  // Compute surface normals and curvature
  PointCloudWithNormals::Ptr points_with_normals_src (new PointCloudWithNormals);
  PointCloudWithNormals::Ptr points_with_normals_tgt (new PointCloudWithNormals);

  pcl::NormalEstimation<PointT, PointNormalT> norm_est;
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
  norm_est.setSearchMethod (tree);
  norm_est.setKSearch (30);
  
  norm_est.setInputCloud (src);
  norm_est.compute (*points_with_normals_src);
  pcl::copyPointCloud (*src, *points_with_normals_src);

  norm_est.setInputCloud (tgt);
  norm_est.compute (*points_with_normals_tgt);
  pcl::copyPointCloud (*tgt, *points_with_normals_tgt);

  //
  // Instantiate our custom point representation (defined above) ...
  KTM::PointAndCurvatureRepresentation point_representation;
  // ... and weight the 'curvature' dimension so that it is balanced against x, y, and z
  float alpha[4] = {1.0, 1.0, 1.0, 1.0};
  point_representation.setRescaleValues (alpha);

  //
  // Align
  pcl::IterativeClosestPointNonLinear<PointNormalT, PointNormalT> reg;
  reg.setTransformationEpsilon (1e-6);
  // Set the maximum distance between two correspondences (src<->tgt) to 10cm
  // Note: adjust this based on the size of your datasets
  reg.setMaxCorrespondenceDistance (0.1);  
  // Set the point representation
  reg.setPointRepresentation (boost::make_shared<const KTM::PointAndCurvatureRepresentation> (point_representation));

  reg.setInputCloud (points_with_normals_src);
  reg.setInputTarget (points_with_normals_tgt);



  //
  // Run the same optimization in a loop and visualize the results
  Eigen::Matrix4f Ti = Eigen::Matrix4f::Identity (), prev, targetToSource;
  PointCloudWithNormals::Ptr reg_result = points_with_normals_src;
  reg.setMaximumIterations (2);
  for (int i = 0; i < 30; ++i)
  {
    PCL_INFO ("Iteration Nr. %d.\n", i);

    // save cloud for visualization purpose
    points_with_normals_src = reg_result;

    // Estimate
    reg.setInputCloud (points_with_normals_src);
    reg.align (*reg_result);

		//accumulate transformation between each Iteration
    Ti = reg.getFinalTransformation () * Ti;

		//if the difference between this transformation and the previous one
		//is smaller than the threshold, refine the process by reducing
		//the maximal correspondence distance
    if (fabs ((reg.getLastIncrementalTransformation () - prev).sum ()) < reg.getTransformationEpsilon ())
      reg.setMaxCorrespondenceDistance (reg.getMaxCorrespondenceDistance () - 0.001);
    
    prev = reg.getLastIncrementalTransformation ();
  }
}