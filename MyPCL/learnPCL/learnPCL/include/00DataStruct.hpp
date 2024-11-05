#pragma once
#include <pcl/io/pcd_io.h>

namespace PCLdataStruct
{
	void demo1()
	{
		//常见的点云创建形式
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPtr(new pcl::PointCloud<pcl::PointXYZ>);
		pcl::PointCloud<pcl::PointXYZ> cloud;
		pcl::PointXYZ point;

		//访问形式
		auto x = cloud.points[0].x;	//第一个点的x
		x = cloudPtr->points[0].x;
		x = point.x;
	}
	
}