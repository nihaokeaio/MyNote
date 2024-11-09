#pragma once
#include "commonInclude.h"

namespace Chapter00
{
	class CreateCloud
	{

	public:

		static auto createRadomPointCloud(int width, int height, const std::string& saveName = "")
			-> pcl::PointCloud<pcl::PointXYZ>::Ptr;

		static void printCloud(const pcl::PointCloud<pcl::PointXYZ>* pCloud, unsigned int size = 10);
	};
}
