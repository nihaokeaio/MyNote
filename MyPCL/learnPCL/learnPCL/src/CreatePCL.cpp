#include "CreatePCL.h"

auto Chapter00::CreateCloud::createRadomPointCloud(int width, int height, const std::string& saveName)
-> pcl::PointCloud<pcl::PointXYZ>::Ptr
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	cloud->width = width;
	cloud->height = height;
	cloud->is_dense = false;
	cloud->resize(cloud->width * cloud->height);
	for (auto& point : *cloud)
	{
		point.x = 1024 * rand() / (RAND_MAX + 1.0f) / 1000.f;
		point.y = 1024 * rand() / (RAND_MAX + 1.0f) / 1000.f;
		point.z = 1024 * rand() / (RAND_MAX + 1.0f) / 1000.f;
	}

	if (!saveName.empty())
	{
		pcl::io::savePCDFileASCII(saveName, *cloud);
	}
	return cloud;
}

void Chapter00::CreateCloud::printCloud(const pcl::PointCloud<pcl::PointXYZ>* pCloud, unsigned int size)
{
	for (size_t i = 0; i < size; ++i)
		std::cout << "    " << pCloud->points[i].x
		<< " " << pCloud->points[i].y
		<< " " << pCloud->points[i].z
		<< std::endl;
}
