#pragma once
#include "commonInclude.h"
#include <pcl/octree/octree.h>

class OcTree
{
public:
	OcTree(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);

	//���ؽ���������ȡ����OcTree�ķֱ��ʣ����ȣ�
	bool queryVoxel(const pcl::PointXYZ& point, std::vector<int>& indexList);

	//k��������
	bool queryKNearest(const pcl::PointXYZ& point, int k, std::vector<int>& indexList, std::vector<float>& distList);

	//��Χ����
	bool queryRadius(const pcl::PointXYZ& point, float radius, std::vector<int>& indexList, std::vector<float>& distList);

	static void ocTreeChangeDetection();
private:
	pcl::octree::OctreePointCloudSearch<pcl::PointXYZ>::Ptr octree_;
};