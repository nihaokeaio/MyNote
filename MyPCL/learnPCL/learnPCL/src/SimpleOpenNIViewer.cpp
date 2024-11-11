#include "SimpleOpenNIViewer.h"
#include <pcl/io/openni2_grabber.h>

SimpleOpenNIViewer::SimpleOpenNIViewer():viewer_("Point Cloud Compression Example")
{
}

void SimpleOpenNIViewer::cloudCB(const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr& pCloud)
{
	if (!viewer_.wasStopped())
	{		
		std::stringstream compressedData;

		pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloutOut(new pcl::PointCloud<pcl::PointXYZRGBA>);

		PointCloudEncoder->encodePointCloud(pCloud, compressedData);
		PointCloudDecoder->decodePointCloud(compressedData, cloutOut);
		
	}
}

void SimpleOpenNIViewer::run()
{
	bool showStatistics = true; //设置在标准设备上输出打印出压缩结果信息

	// 压缩选项详情在: /io/include/pcl/compression/compression_profiles.h
	pcl::io::compression_Profiles_e compressionProfile = pcl::io::MED_RES_ONLINE_COMPRESSION_WITH_COLOR;

	PointCloudEncoder.reset(new pcl::io::OctreePointCloudCompression<pcl::PointXYZRGBA>(compressionProfile, showStatistics));
	PointCloudDecoder.reset(new pcl::io::OctreePointCloudCompression<pcl::PointXYZRGBA>());

	//不知为何，构造对象会崩溃
	std::shared_ptr<pcl::Grabber> interface(new pcl::io::OpenNI2Grabber);
	const std::function<void(const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr&)>& f = std::bind(&SimpleOpenNIViewer::cloudCB, this, std::placeholders::_1);
	boost::signals2::connection c = interface->registerCallback(f);
	interface->start();

	while (!viewer_.wasStopped())
	{
		Sleep(1);
	}

	interface->stop();
}
