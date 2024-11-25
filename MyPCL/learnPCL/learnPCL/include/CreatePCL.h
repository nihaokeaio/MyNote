#pragma once
#include "commonInclude.h"

namespace Chapter00
{
	class CreateCloud
	{

	public:
		//����һ��ָ����С���������
		static auto createRandomPointCloud(int width, int height, const std::string& saveName = "")
			-> pcl::PointCloud<pcl::PointXYZ>::Ptr;

		//����һ�������������ε���
		static auto createRandomSpherePointCloud(const Eigen::Vector3f& center, float radius, const std::string& saveName = "", 
			bool withNoise = false)
			-> pcl::PointCloud<pcl::PointXYZ>::Ptr;

		//����һ����������ƽ�����
		static auto createRandomPlanePointCloud(const Eigen::Vector3f& center, const Eigen::Vector3f& normal
			, const std::string& saveName = "", bool withNoise = false)
			-> pcl::PointCloud<pcl::PointXYZ>::Ptr;


		static void printCloud(const pcl::PointCloud<pcl::PointXYZ>* pCloud, unsigned int size = 10);

		static pcl::PolygonMesh loadPLYFile(const std::string& filePath);
	};
}
