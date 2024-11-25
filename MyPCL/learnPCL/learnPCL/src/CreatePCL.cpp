#include "CreatePCL.h"
#include <pcl/common/angles.h>
#include <pcl/io/ply_io.h>

auto Chapter00::CreateCloud::createRandomPointCloud(int width, int height, const std::string& saveName)
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

auto Chapter00::CreateCloud::createRandomSpherePointCloud(const Eigen::Vector3f& center, float radius, 
	const std::string& saveName, bool withNoise) -> pcl::PointCloud<pcl::PointXYZ>::Ptr
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	cloud->width = 50;
	cloud->height = 50;
	cloud->is_dense = false;
	cloud->resize(cloud->width * cloud->height);
	int count = 0;
	for (int i = 0; i < cloud->size(); ++i)
	{
		float theta = rand() / (RAND_MAX + 1.0) * 2 * M_PI;
		float phi = rand() / (RAND_MAX + 1.0) * 2 * M_PI;
		float x, y, z;
		if (i % 5 != 0 && withNoise)
		{
			x = 2 * radius * rand() / (RAND_MAX + 1.0) - radius;
			y = 2 * radius * rand() / (RAND_MAX + 1.0) - radius;
			z = 2 * radius * rand() / (RAND_MAX + 1.0) - radius;
		}
		else
		{
			// 球形坐标转换为笛卡尔坐标
			x = radius * sin(phi) * cos(theta);
			y = radius * sin(phi) * sin(theta);
			z = radius * cos(phi);
			count++;
		}
		// 将点添加到点云中

		cloud->points[i].x = x;
		cloud->points[i].y = y;
		cloud->points[i].z = z;
	}

	if (!saveName.empty())
	{
		pcl::io::savePCDFileASCII(saveName, *cloud);
	}

	return cloud;
}

auto Chapter00::CreateCloud::createRandomPlanePointCloud(const Eigen::Vector3f& center, const Eigen::Vector3f& normal, 
	const std::string& saveName, bool withNoise) -> pcl::PointCloud<pcl::PointXYZ>::Ptr
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	cloud->width = 500;
	cloud->height = 500;
	cloud->is_dense = false;
	cloud->resize(cloud->width * cloud->height);
	for (int i = 0; i < cloud->size(); ++i)
	{
		cloud->points[i].x = rand() / (RAND_MAX + 1.0);
		cloud->points[i].y = rand() / (RAND_MAX + 1.0);
		if (i % 5 != 0 && withNoise)
			cloud->points[i].z = rand() / (RAND_MAX + 1.0);   //对应的局外点
		else
			cloud->points[i].z = (normal.transpose() * center - normal.x() * cloud->points[i].x - normal.y() * cloud->points[i].y) / (normal.z() + 0.001);
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

pcl::PolygonMesh Chapter00::CreateCloud::loadPLYFile(const std::string& filePath)
{
	/*如果表头element face的值为0, 则表示该文件为点云文件*/
	pcl::PolygonMesh mesh;
	pcl::io::loadPLYFile("readName.ply", mesh);
	pcl::io::savePLYFile("saveName.ply", mesh);
	return mesh;
}
