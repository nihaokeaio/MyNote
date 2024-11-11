#pragma once
#include <pcl/io/pcd_io.h>
#include <pcl/common/transforms.h>

namespace Base00
{
	class ChapterDemo
	{
	public:
		static void demo1()
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

		static void transFormCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr& inCloud, pcl::PointCloud<pcl::PointXYZ>::Ptr& outCloud)
		{
			if(outCloud)
				outCloud->clear();
			outCloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
			pcl::copyPointCloud(*inCloud, *outCloud);

			// 创建3x3单位阵
			float theta = M_PI_4;
			Eigen::Affine3f transform_2 = Eigen::Affine3f::Identity();

			transform_2.translation() << 5.5, 0.0, 0.0;

			transform_2.rotate(Eigen::AngleAxisf(theta, Eigen::Vector3f::UnitZ()));

			//std::cout << transform_2.matrix() << std::endl;

			pcl::transformPointCloud(*inCloud, *outCloud, transform_2);
		}
	};	
}