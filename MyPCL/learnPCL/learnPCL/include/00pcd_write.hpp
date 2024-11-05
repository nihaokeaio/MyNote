#pragma once
#include <pcl/io/pcd_io.h>

namespace Space00
{
	void foo()
	{
		pcl::PointCloud<pcl::PointXYZ> cloud;
		cloud.width = 5;
		cloud.height = 1;
		cloud.is_dense = false;
		cloud.resize(cloud.width * cloud.height);
		for (auto& point : cloud)
		{
			point.x = 1024 * rand() / (RAND_MAX + 1.0f) / 1000.f;
			point.y = 1024 * rand() / (RAND_MAX + 1.0f) / 1000.f;
			point.z = 1024 * rand() / (RAND_MAX + 1.0f) / 1000.f;
		}
		pcl::io::savePCDFileASCII("test.pcd", cloud);
		std::cerr << "Saved " << cloud.size() << " data points to test_pcd.pcd." << std::endl;

		for (const auto& point : cloud)
			std::cerr << "    " << point.x << " " << point.y << " " << point.z << std::endl;
	}

	void run()
	{
		foo();
	}
}