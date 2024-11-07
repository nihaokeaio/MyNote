#include "CreatePCL.h"

auto Chapter00::CreateCloud::createRadomPointCloud(const std::string& saveName) -> pcl::PointCloud<pcl::PointXYZ>::Ptr
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	cloud->width = 500;
	cloud->height = 200;
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
