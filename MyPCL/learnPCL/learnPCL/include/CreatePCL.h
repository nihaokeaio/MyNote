#pragma once
#include "commonInclude.h"

namespace Chapter00
{
	class CreateCloud
	{

	public:
		//创建一个指定大小的随机点云
		static auto createRandomPointCloud(int width, int height, const std::string& saveName = "")
			-> pcl::PointCloud<pcl::PointXYZ>::Ptr;

		//创建一个带噪声的球形点云
		static auto createRandomSpherePointCloud(const Eigen::Vector3f& center, float radius, const std::string& saveName = "", 
			bool withNoise = false)
			-> pcl::PointCloud<pcl::PointXYZ>::Ptr;

		//创建一个带噪声的平面点云
		static auto createRandomPlanePointCloud(const Eigen::Vector3f& center, const Eigen::Vector3f& normal
			, const std::string& saveName = "", bool withNoise = false)
			-> pcl::PointCloud<pcl::PointXYZ>::Ptr;


		static void printCloud(const pcl::PointCloud<pcl::PointXYZ>* pCloud, unsigned int size = 10);

		static pcl::PolygonMesh loadPLYFile(const std::string& filePath);
	};
}
