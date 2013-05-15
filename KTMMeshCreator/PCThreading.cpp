#include "PCThreading.hpp"

//----------------------------------------------------------------------------
// Depth Array to PCL Point Cloud
//----------------------------------------------------------------------------
void KTM::PCThreading::depthArrayToPointCloud(unsigned short* depthData, pcl::PointCloud<pcl::PointXYZ>::Ptr transformationMatrix, PointCloud::Ptr outCloud, int numThreads){
	pthread_t* threads = new pthread_t[numThreads];
	PCThreading::depthArrayToPointCloudArgs* args = new PCThreading::depthArrayToPointCloudArgs[numThreads];
	pthread_attr_t attr;
	pthread_attr_init(&attr);
	pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_JOINABLE);
	
	for(int i = 0; i < numThreads; i++){
		args[i].numThreads = numThreads;
		args[i].threadIndex = i;
		args[i].outCloud = outCloud;
		args[i].depthData = depthData;
		args[i].transformationMatrix = transformationMatrix;
		pthread_create(&threads[i], &attr, PCThreading::t_depthArrayToPointCloud, (void*)&args[i]);
	}

	for(int i = 0; i < numThreads; i++){
		pthread_join(threads[i], NULL);
	}

	delete[] args;
	delete[] threads;
}

void* KTM::PCThreading::t_depthArrayToPointCloud(void* threadArgs){
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

//----------------------------------------------------------------------------
// Depth Array to Vector Point Cloud - Kinect Transformation Matrix
//----------------------------------------------------------------------------
void KTM::PCThreading::depthArrayToVectorPC(unsigned short* depthData, int width, int height, pcl::PointCloud<pcl::PointXYZ>::Ptr transformationMatrix, Eigen::Vector4f* outVector, int numThreads){
	pthread_t* threads = new pthread_t[numThreads];
	depthArrayToVectorPCArgs* args = new depthArrayToVectorPCArgs[numThreads];
	pthread_attr_t attr;
	pthread_attr_init(&attr);
	pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_JOINABLE);
	
	for(int i = 0; i < numThreads; i++){
		args[i].numThreads = numThreads;
		args[i].threadIndex = i;
		args[i].depthData = depthData;
		args[i].width = width;
		args[i].height = height;
		args[i].transformationMatrix = transformationMatrix;
		args[i].outPoints = outVector;
		pthread_create(&threads[i], &attr, t_depthArrayToVectorPC, (void*)&args[i]);
	}

	for(int i = 0; i < numThreads; i++){
		pthread_join(threads[i], NULL);
	}

	delete[] args;
	delete[] threads;
}

void* KTM::PCThreading::t_depthArrayToVectorPC(void* threadArgs){
	depthArrayToVectorPCArgs* args = (depthArrayToVectorPCArgs*)threadArgs;
	int pointsPerThread = args->transformationMatrix->size() / args->numThreads;

	for(int i = args->threadIndex * pointsPerThread; i < (args->threadIndex + 1) * pointsPerThread && i < args->transformationMatrix->size(); i++){
		if(args->depthData[i] > 0){
			args->outPoints[i].x() = args->transformationMatrix->points[i].x * (float)args->depthData[i];
			args->outPoints[i].y() = args->transformationMatrix->points[i].y * (float)args->depthData[i];
			args->outPoints[i].z() = args->transformationMatrix->points[i].z * (float)args->depthData[i];
			args->outPoints[i].w() = 1;
		}else{
			args->outPoints[i].x() = 0;
			args->outPoints[i].y() = 0;
			args->outPoints[i].z() = 0;
			args->outPoints[i].w() = 0;
		}
	}
	return NULL;
}

//----------------------------------------------------------------------------
// Depth Array to Vector Point Cloud
//----------------------------------------------------------------------------
void KTM::PCThreading::depthArrayToVectorPC(unsigned short* depthData, int width, int height, float* TAN, Eigen::Vector4f* outVector, int numThreads){
	pthread_t* threads = new pthread_t[numThreads];
	depthArrayToVectorPCTANArgs* args = new depthArrayToVectorPCTANArgs[numThreads];
	pthread_attr_t attr;
	pthread_attr_init(&attr);
	pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_JOINABLE);
	
	for(int i = 0; i < numThreads; i++){
		args[i].numThreads = numThreads;
		args[i].threadIndex = i;
		args[i].depthData = depthData;
		args[i].width = width;
		args[i].height = height;
		args[i].TAN = TAN;
		args[i].outPoints = outVector;
		pthread_create(&threads[i], &attr, t_depthArrayToVectorPCTAN, (void*)&args[i]);
	}

	for(int i = 0; i < numThreads; i++){
		pthread_join(threads[i], NULL);
	}

	delete[] args;
	delete[] threads;
}

void* KTM::PCThreading::t_depthArrayToVectorPCTAN(void* threadArgs){
	depthArrayToVectorPCTANArgs* args = (depthArrayToVectorPCTANArgs*)threadArgs;

	int linesPerThread = args->height / args->numThreads;
	float tID = args->threadIndex;
	float startLine = tID * linesPerThread;
	float endLine;

	if(tID == args->numThreads - 1)
		endLine = startLine + ((tID + 1) * linesPerThread);
	else
		endLine = startLine;

	for(int i = startLine; i < endLine; i++){
		for(int j = 0; j < args->width; j++){
			int index = (i * args->width) + j;
			args->outPoints[index](3) = 1;
			if(args->depthData == 0){
				continue;
				args->outPoints[index].x() = 0;
				args->outPoints[index].y() = 0;
				args->outPoints[index].z() = 0;
			}else{
				args->outPoints[index].z() = acos(atan(args->TAN[i])) * args->depthData[index];
				args->outPoints[index].y() = args->outPoints[index].z() * args->TAN[i];
				args->outPoints[index].x() = args->outPoints[index].y() / args->outPoints[index].z();
			}
		}
	}

	return NULL;
}

//----------------------------------------------------------------------------
// Create TAN Table
//----------------------------------------------------------------------------
void KTM::PCThreading::createTANTable(int fieldHeight, float verticalFOV, float* tan, int numThreads){
	pthread_t* threads = new pthread_t[numThreads];
	createTANTableArgs* args = new createTANTableArgs[numThreads];
	pthread_attr_t attr;
	pthread_attr_init(&attr);
	pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_JOINABLE);
	
	for(int i = 0; i < numThreads; i++){
		args[i].numThreads = numThreads;
		args[i].threadIndex = i;
		args[i].TAN = tan;
		args[i].verticalFOV = verticalFOV;
		args[i].fieldHeight = fieldHeight;
		pthread_create(&threads[i], &attr, t_createTANTable, (void*)&args[i]);
	}

	for(int i = 0; i < numThreads; i++){
		pthread_join(threads[i], NULL);
	}

	delete[] args;
	delete[] threads;
};

void* KTM::PCThreading::t_createTANTable(void* threadArgs){
	createTANTableArgs* args = (createTANTableArgs*)threadArgs;
	int linesPerThread = args->fieldHeight / args->numThreads;
	float topAngle = args->verticalFOV / 2.0f;
	float bottomAngle = -topAngle;
	float anglePerLine = args->verticalFOV / (float)args->fieldHeight;
	float tID = args->threadIndex;
	float startLine = tID * linesPerThread;
	float endLine;

	if(tID == args->numThreads - 1)
		endLine = startLine + ((tID + 1) * linesPerThread);
	else
		endLine = startLine;

	for(int i = startLine; i < endLine; i++){
		args->TAN[i] = tan(bottomAngle + (anglePerLine * i));
	}

	return NULL;
}

//----------------------------------------------------------------------------
// Normals from Vectors
//----------------------------------------------------------------------------
void KTM::PCThreading::generateNormalsFromVectors(Eigen::Vector4f* points, int width, int height, Eigen::Vector4f* outNormals, int numThreads){
	pthread_t* threads = new pthread_t[numThreads];
	generateNormalsFromVectorArgs* args = new generateNormalsFromVectorArgs[numThreads];
	pthread_attr_t attr;
	pthread_attr_init(&attr);
	pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_JOINABLE);
	
	for(int i = 0; i < numThreads; i++){
		args[i].numThreads = numThreads;
		args[i].threadIndex = i;
		args[i].points = points;
		args[i].width = width;
		args[i].height = height;
		args[i].outNormals = outNormals;
		pthread_create(&threads[i], &attr, t_generateNormalsFromVectors, (void*)&args[i]);
	}

	for(int i = 0; i < numThreads; i++){
		pthread_join(threads[i], NULL);
	}

	delete[] args;
	delete[] threads;
}

void* KTM::PCThreading::t_generateNormalsFromVectors(void* threadArgs){
	generateNormalsFromVectorArgs* args = (generateNormalsFromVectorArgs*)threadArgs;

	int linesPerThread = args->height / args->numThreads;
	float tID = args->threadIndex;
	float startLine = tID * linesPerThread;
	float endLine;

	if(tID == args->numThreads - 1)
		endLine = startLine + ((tID + 1) * linesPerThread);
	else
		endLine = startLine;

	for(int y = startLine; y < args->height; y++){
		for(int x = 0; x < args->width; x++){
			int index = (y * args->width) + x;

			int indexV2 = x < args->width - 1 ? x + 1 : x - 1;
			indexV2 = (y * args->width) + indexV2;

			int indexV3 = y < args->height - 1 ? y + 1 : y - 1;
			indexV3 = (indexV3 * args->width) + x;

			Eigen::Vector3f v1(args->points[index].x(), args->points[index].y(), args->points[index].z());
			Eigen::Vector3f v2(args->points[indexV2].x(), args->points[indexV2].y(), args->points[indexV2].z());
			Eigen::Vector3f v3(args->points[indexV3].x(), args->points[indexV3].y(), args->points[indexV3].z());
			try{
				Eigen::Vector3f n = (v2 - v1).cross(v3 - v1);
				n.normalize();
				args->outNormals[index](0) = n.x();
				args->outNormals[index](1) = n.y();
				args->outNormals[index](2) = n.z();
				args->outNormals[index](3) = 0;
			}catch(...){
				args->outNormals[index](0) = 0;
				args->outNormals[index](1) = 0;
				args->outNormals[index](2) = 0;
				args->outNormals[index](3) = 0;
			}
			
		}
	}

	return NULL;
}

//----------------------------------------------------------------------------
// Depth Array to PCL Point Cloud
//----------------------------------------------------------------------------
void KTM::PCThreading::vectorPCToPCLPC(Eigen::Vector4f* points, int size, PointCloud::Ptr outCloud, int numThreads){
	pthread_t* threads = new pthread_t[numThreads];
	PCThreading::_vectorPCToPCLPCArgs* args = new PCThreading::_vectorPCToPCLPCArgs[numThreads];
	pthread_attr_t attr;
	pthread_attr_init(&attr);
	pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_JOINABLE);
	
	for(int i = 0; i < numThreads; i++){
		args[i].numThreads = numThreads;
		args[i].threadIndex = i;
		args[i].size = size;
		args[i].points = points;
		args[i].outCloud = outCloud;
		pthread_create(&threads[i], &attr, PCThreading::t_vectorPCToPCLPC, (void*)&args[i]);
	}

	for(int i = 0; i < numThreads; i++){
		pthread_join(threads[i], NULL);
	}

	delete[] args;
	delete[] threads;
}

void* KTM::PCThreading::t_vectorPCToPCLPC(void* threadArgs){
	_vectorPCToPCLPCArgs* args = (_vectorPCToPCLPCArgs*)threadArgs;
	int pointsPerThread = args->size / args->numThreads;

	for(int i = args->threadIndex * pointsPerThread; i < (args->threadIndex + 1) * pointsPerThread; i++){
		if(args->points[i].x() != 0 || args->points[i].y() != 0 || args->points[i].z() != 0){
			args->outCloud->points[i].x = args->points[i].x();
			args->outCloud->points[i].y = args->points[i].y();
			args->outCloud->points[i].z = args->points[i].z();
			args->outCloud->points[i].r = 255.0f;
			args->outCloud->points[i].g = 255.0f;
			args->outCloud->points[i].b = 255.0f;
		}
	}
	return NULL;
}