#pragma once
#include <pcl/io/pcd_io.h>

namespace Chapter00
{
	class CreateCloud
	{

	public:

		static auto createRadomPointCloud(const std::string& saveName = "") -> pcl::PointCloud<pcl::PointXYZ>::Ptr;




	};
}
