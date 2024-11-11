#pragma once
#include "commonInclude.h"
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/compression/octree_pointcloud_compression.h>


class SimpleOpenNIViewer
{
public:
	SimpleOpenNIViewer();

	void cloudCB(const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr& pCloud);

	void run();
private:
	pcl::visualization::CloudViewer viewer_;

	pcl::io::OctreePointCloudCompression<pcl::PointXYZRGBA>::Ptr PointCloudEncoder;
	pcl::io::OctreePointCloudCompression<pcl::PointXYZRGBA>::Ptr PointCloudDecoder;
};