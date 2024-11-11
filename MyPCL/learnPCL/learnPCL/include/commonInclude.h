#pragma once


#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/kdtree/kdtree_flann.h> //kdtree类定义头文件

namespace MyColor
{
	const Eigen::Vector4f Red = Eigen::Vector4f(1.0, 0.0, 0.0, 1.0);
	const Eigen::Vector4f Green = Eigen::Vector4f(0.0, 1.0, 0.0, 1.0);
	const Eigen::Vector4f Blue = Eigen::Vector4f(0.0, 0.0, 1.0, 1.0);
	const Eigen::Vector4f White = Eigen::Vector4f(1.0, 1.0, 1.0, 1.0);
	const Eigen::Vector4f Black = Eigen::Vector4f(0.0, 0.0, 0.0, 1.0);
};
