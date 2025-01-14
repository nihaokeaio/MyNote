#pragma once

#include "commonInclude.h"
#include "ShowDebug.h"

class FilterBase
{
public:
	FilterBase();

	pcl::PointCloud<pcl::PointXYZ>::Ptr passThrought(const pcl::PointCloud<pcl::PointXYZ>::Ptr& pCloud);

private:
	std::shared_ptr<ShowDebug> viewer_;
};