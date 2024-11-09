#pragma once

#include "commonInclude.h"

class KDTreeBase
{
	using KDTreePtr = pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr;
public:
	bool createKDTree(const pcl::PointCloud<pcl::PointXYZ>::Ptr& pCloud);

	//目前只支持float
	bool query(const pcl::PointXYZ& point, int k, std::vector<int>& indexList, std::vector<float>& distList);

	//范围查找
	bool queryRadius(const pcl::PointXYZ& point, float radius, std::vector<int>& indexList, std::vector<float>& distList);

	void demoQuery();

	void demoQueryRadius();
private:
	KDTreePtr kdTree;
};
