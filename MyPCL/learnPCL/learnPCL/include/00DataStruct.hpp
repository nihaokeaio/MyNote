#pragma once
#include <pcl/io/pcd_io.h>

namespace PCLdataStruct
{
	void demo1()
	{
		//�����ĵ��ƴ�����ʽ
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPtr(new pcl::PointCloud<pcl::PointXYZ>);
		pcl::PointCloud<pcl::PointXYZ> cloud;
		pcl::PointXYZ point;

		//������ʽ
		auto x = cloud.points[0].x;	//��һ�����x
		x = cloudPtr->points[0].x;
		x = point.x;
	}
	
}