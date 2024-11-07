#include <ShowDebug.h>

ShowDebug::ShowDebug()
{
	viewer_.setWindowName("viewer");
	viewer_.addCoordinateSystem(1.0, "cloud", 0);
	viewer_.setBackgroundColor(0.01, 0.01, 0.01, 0); // Setting background to a dark grey
	/*viewer_.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "original_cloud");
	viewer_.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "transformed_cloud");*/
}

void ShowDebug::addPoints(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, const std::string& name)
{
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> source_cloud_color_handler(cloud, 255, 255, 255);
	viewer_.addPointCloud(cloud, source_cloud_color_handler, name);
}

void ShowDebug::run()
{
	while (!viewer_.wasStopped())
	{ // Display the visualiser until 'q' key is pressed
		viewer_.spinOnce();
	}
}
