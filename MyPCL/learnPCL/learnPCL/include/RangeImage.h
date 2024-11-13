#pragma once
#include <commonInclude.h>

class ShowDebug;

namespace pcl
{
	class RangeImage;
}

class RangeImage
{
public:
	RangeImage();

	void demo();

	std::shared_ptr<pcl::RangeImage> createRangeImage(const pcl::PointCloud<pcl::PointXYZ>::Ptr& pCloud);

private:
	std::shared_ptr<ShowDebug> viewer_;
};