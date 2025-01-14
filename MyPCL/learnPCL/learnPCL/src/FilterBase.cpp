#include "FilterBase.h"
#include <pcl/filters/passthrough.h>

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
	pass.setFilterLimits(0.0, 1.0);        //设置在过滤字段的范围
	//pass.setFilterLimitsNegative (true);   //设置保留范围内还是过滤掉范围内 是否保存滤波的限制范围内的点云，默认为false，保存限制范围内点云，true时候是相反。
	pass.filter(*retPoints);            //执行滤波，保存过滤结果在cloud_filtered

	return retPoints;
}
