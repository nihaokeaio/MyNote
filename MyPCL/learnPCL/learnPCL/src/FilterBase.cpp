#include "FilterBase.h"
#include <pcl/filters/passthrough.h>

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
	pass.setFilterLimits(0.0, 1.0);        //�����ڹ����ֶεķ�Χ
	//pass.setFilterLimitsNegative (true);   //���ñ�����Χ�ڻ��ǹ��˵���Χ�� �Ƿ񱣴��˲������Ʒ�Χ�ڵĵ��ƣ�Ĭ��Ϊfalse���������Ʒ�Χ�ڵ��ƣ�trueʱ�����෴��
	pass.filter(*retPoints);            //ִ���˲���������˽����cloud_filtered

	return retPoints;
}
