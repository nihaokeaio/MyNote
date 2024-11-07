#pragma once
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>


class ShowDebug
{
public:
	ShowDebug();

	void addPoints(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, const std::string& name = "cloud");

	void run();
private:
	pcl::visualization::PCLVisualizer viewer_;
};