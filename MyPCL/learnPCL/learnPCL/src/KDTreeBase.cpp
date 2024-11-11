#include "KDTreeBase.h"
#include "CreatePCL.h"

bool KDTreeBase::createKDTree(const pcl::PointCloud<pcl::PointXYZ>::Ptr& pCloud)
{
	if (!pCloud || pCloud->empty())
		return false;

	//pcl::KdTreeFLANN<PointT>是pcl::KdTree<PointT>的子类，可以实现同样的功能。
	kdTree.reset(new pcl::KdTreeFLANN<pcl::PointXYZ>);
	kdTree->setInputCloud(pCloud);
}

bool KDTreeBase::query(const pcl::PointXYZ& point, int k, std::vector<int>& indexList, std::vector<float>& distList)
{
	indexList.resize(k, 0);
	distList.resize(k, 0);
	if (kdTree->nearestKSearch(point, k, indexList, distList) > 0)
	{
		return true;
	}
	return false;
}

bool KDTreeBase::queryRadius(const pcl::PointXYZ& point, float radius, std::vector<int>& indexList, std::vector<float>& distList)
{
	if (kdTree->radiusSearch(point, radius, indexList, distList) > 0)
	{
		return true;
	}
	return false;
}

void KDTreeBase::demoQuery()
{
	auto cloud = Chapter00::CreateCloud::createRandomPointCloud(200, 100, "randCloud.pcd");

	pcl::PointXYZ point(5.0, 5.0, 5.0);

	KDTreeBase kdTreeBase;
	kdTreeBase.createKDTree(cloud);

	std::vector<int> indexList;
	std::vector<float> distList;
	kdTreeBase.query(point, 10, indexList, distList);

	for (size_t i = 0; i < 10; ++i)
		std::cout << "    " << cloud->points[indexList[i]].x
		<< " " << cloud->points[indexList[i]].y
		<< " " << cloud->points[indexList[i]].z
		<< " (squared distance: " << distList[i] << ")" << std::endl;
}
