#pragma once
#include "commonInclude.h"
#include <pcl/octree/octree.h>

class OcTree
{
public:
	OcTree(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);

	//体素近邻搜索，取决于OcTree的分辨率（精度）
	bool queryVoxel(const pcl::PointXYZ& point, std::vector<int>& indexList);

	//k近邻搜索
	bool queryKNearest(const pcl::PointXYZ& point, int k, std::vector<int>& indexList, std::vector<float>& distList);

	//范围查找
	bool queryRadius(const pcl::PointXYZ& point, float radius, std::vector<int>& indexList, std::vector<float>& distList);

	static void ocTreeChangeDetection();
private:
	pcl::octree::OctreePointCloudSearch<pcl::PointXYZ>::Ptr octree_;
};