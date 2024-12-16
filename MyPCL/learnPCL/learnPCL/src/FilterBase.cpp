#include "FilterBase.h"
#include "CreatePCL.h"
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/io.h>

enum class FilterType
{
	PASSTHROUGHT,
	VOXELFILTER
};

FilterBase::FilterBase()
{
	viewer_.reset(new ShowDebug("RangeImage"));
}

pcl::PointCloud<pcl::PointXYZ>::Ptr FilterBase::passThrought(const pcl::PointCloud<pcl::PointXYZ>::Ptr& pCloud)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr retPoints(new pcl::PointCloud<pcl::PointXYZ>);
	// 设置滤波器对象
	pcl::PassThrough<pcl::PointXYZ> pass;
	pass.setInputCloud(pCloud);            //设置输入点云
	pass.setFilterFieldName("z");         //设置过滤时所需要点云类型的Z字段
	pass.setFilterLimits(0.4, 0.6);        //设置在过滤字段的范围
	//pass.setFilterLimitsNegative (true);   //设置保留范围内还是过滤掉范围内 是否保存滤波的限制范围内的点云，默认为false，保存限制范围内点云，true时候是相反。
	pass.filter(*retPoints);            //执行滤波，保存过滤结果在cloud_filtered

	return retPoints;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr FilterBase::voxelGridFilter(const pcl::PointCloud<pcl::PointXYZ>::Ptr& pCloud)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr retPoints(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::VoxelGrid<pcl::PointXYZ> sor;
	sor.setInputCloud(pCloud);
	sor.setLeafSize(0.01f, 0.01, 0.01);
	sor.filter(*retPoints);
	return retPoints;
}



void FilterBase::demo()
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr originCloud;
	pcl::PointCloud<pcl::PointXYZ>::Ptr filterCloud;

	auto type = FilterType::VOXELFILTER;

	switch (type)
	{
	case FilterType::PASSTHROUGHT:
	{
		originCloud = Chapter00::CreateCloud::createRandomPointCloud(50, 50);
		filterCloud = passThrought(originCloud);
		break;
	}
		
	case FilterType::VOXELFILTER:
	{
		originCloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
		pcl::io::loadPCDFile("table_scene_lms400.pcd", *originCloud);
		filterCloud = voxelGridFilter(originCloud);
		break;
	}		
	default:
		break;
	}
	

	//viewer_->addPoints(originCloud, "borderPCL", MyColor::Green);
	viewer_->addPoints(filterCloud, "passThroughtCloudPCL", MyColor::Blue);

	viewer_->run();
}
