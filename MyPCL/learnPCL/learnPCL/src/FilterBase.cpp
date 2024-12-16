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
	// �����˲�������
	pcl::PassThrough<pcl::PointXYZ> pass;
	pass.setInputCloud(pCloud);            //�����������
	pass.setFilterFieldName("z");         //���ù���ʱ����Ҫ�������͵�Z�ֶ�
	pass.setFilterLimits(0.4, 0.6);        //�����ڹ����ֶεķ�Χ
	//pass.setFilterLimitsNegative (true);   //���ñ�����Χ�ڻ��ǹ��˵���Χ�� �Ƿ񱣴��˲������Ʒ�Χ�ڵĵ��ƣ�Ĭ��Ϊfalse���������Ʒ�Χ�ڵ��ƣ�trueʱ�����෴��
	pass.filter(*retPoints);            //ִ���˲���������˽����cloud_filtered

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
