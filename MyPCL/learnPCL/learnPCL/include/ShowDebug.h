#pragma once
#include "commonInclude.h"
class ShowDebug
{
public:
	ShowDebug();

	void addPoints(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, const std::string& name, const Eigen::Vector4f& rgba);

	void run();

	void demo(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);
private:
	pcl::visualization::PCLVisualizer viewer_;
};