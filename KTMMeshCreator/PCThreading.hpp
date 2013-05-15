#ifndef KTM_PCTHREADING
#define KTM_PCTHREADING

#include <pthread.h>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

namespace KTM{
	namespace PCThreading{
		typedef pcl::PointXYZRGB PointT;
		typedef pcl::PointCloud<PointT> PointCloud;
		typedef pcl::PointNormal PointNormalT;
		typedef pcl::PointCloud<PointNormalT> PointCloudWithNormals;
		#define KTM_PCTHREADING_NUM_THREADS 8

		/* Custom Types */
		typedef struct _depthArrayToPointCloudArgs{
			int numThreads;
			int threadIndex;
			unsigned short* depthData;
			pcl::PointCloud<pcl::PointXYZ>::Ptr transformationMatrix;
			PointCloud::Ptr outCloud;
		} depthArrayToPointCloudArgs;

		typedef struct _createTANTableArgs{
			int numThreads;
			int threadIndex;
			int fieldHeight;
			float verticalFOV;
			float* TAN;
		} createTANTableArgs;

		typedef struct _depthArrayToVectorPCArgs{
			int numThreads;
			int threadIndex;
			unsigned short* depthData;
			int width;
			int height;
			pcl::PointCloud<pcl::PointXYZ>::Ptr transformationMatrix;
			Eigen::Vector4f* outPoints;
		} depthArrayToVectorPCArgs;

		typedef struct _depthArrayToVectorPCTANArgs{
			int numThreads;
			int threadIndex;
			unsigned short* depthData;
			int width;
			int height;
			float* TAN;
			Eigen::Vector4f* outPoints;
		} depthArrayToVectorPCTANArgs;

		typedef struct _generateNormalsFromVectorsArgs{
			int numThreads;
			int threadIndex;
			Eigen::Vector4f* points;
			int width;
			int height;
			Eigen::Vector4f* outNormals;
		} generateNormalsFromVectorArgs;

		typedef struct _vectorPCToPCLPCArgs{
			int numThreads;
			int threadIndex;
			Eigen::Vector4f* points;
			int size;
			PointCloud::Ptr outCloud;
		} vectorPCToPCLPCArgs;

		/* Threaded Functions */
		void* t_depthArrayToPointCloud(void* threadArgs);
		void* t_createTANTable(void* threadArgs);
		void* t_depthArrayToVectorPC(void* threadArgs);
		void* t_depthArrayToVectorPCTAN(void* threadArgs);
		void* t_generateNormalsFromVectors(void* threadArgs);
		void* t_vectorPCToPCLPC(void* threadArgs);

		/* Starter Funcitons */
		void depthArrayToPointCloud(unsigned short* depthData, pcl::PointCloud<pcl::PointXYZ>::Ptr transformationMatrix, PointCloud::Ptr outCloud, int numThreads = KTM_PCTHREADING_NUM_THREADS);
		void createTANTable(int fieldHeight, float verticalFOV, float* tan, int numThreads = KTM_PCTHREADING_NUM_THREADS);
		void depthArrayToVectorPC(unsigned short* depthData, int width, int height, float* TAN, Eigen::Vector4f* outVector, int numThreads = KTM_PCTHREADING_NUM_THREADS);
		void depthArrayToVectorPC(unsigned short* depthData, int width, int height, pcl::PointCloud<pcl::PointXYZ>::Ptr transformationMatrix, Eigen::Vector4f* outVector, int numThreads = KTM_PCTHREADING_NUM_THREADS);
		void generateNormalsFromVectors(Eigen::Vector4f* points, int width, int height, Eigen::Vector4f* outNormals, int numThreads = KTM_PCTHREADING_NUM_THREADS);
		void vectorPCToPCLPC(Eigen::Vector4f* points, int size, PointCloud::Ptr outCloud, int numThreads = KTM_PCTHREADING_NUM_THREADS);
	};
};

#endif