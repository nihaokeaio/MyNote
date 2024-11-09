#include <ShowDebug.h>
#include <Base00.hpp>

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
	//获取点云颜色
	// pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> source_cloud_color_handler(cloud, 0, 255, 0);
	viewer_.addPointCloud(cloud, source_cloud_color_handler, name);
	viewer_.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, name);

	//实现对点云法线的显示
	//viewer_.addPointCloudNormals<pcl::PointXYZRGB, pcl::Normal>(cloud, normals, 10, 0.05, "normals");


	////绘制形状的实例代码，绘制点之间的连线，
	//viewer_.addLine(cloud->points[0],
	//		cloud->points[cloud->size() - 1], "line");
	////添加点云中第一个点为中心，半径为0.2的球体，同时可以自定义颜色
	//viewer_.addSphere(cloud->points[0], 0.2, 0.5, 0.5, 0.0, "sphere");

	////---------------------------------------
	////-----Add shapes at other locations添加绘制平面使用标准平面方程ax+by+cz+d=0来定义平面，这个平面以原点为中心，方向沿着Z方向-----
	////---------------------------------------
	//pcl::ModelCoefficients coeffs;
	//coeffs.values.push_back(0.0);
	//coeffs.values.push_back(0.0);
	//coeffs.values.push_back(1.0);
	//coeffs.values.push_back(0.0);
	//viewer_.addPlane(coeffs, "plane");
	////添加锥形的参数
	//coeffs.values.clear();
	//coeffs.values.push_back(0.3);
	//coeffs.values.push_back(0.3);
	//coeffs.values.push_back(0.0);
	//coeffs.values.push_back(0.0);
	//coeffs.values.push_back(1.0);
	//coeffs.values.push_back(0.0);
	//coeffs.values.push_back(5.0);
	//viewer_.addCone(coeffs, "cone");

}

void ShowDebug::run()
{
	while (!viewer_.wasStopped())
	{ // Display the visualiser until 'q' key is pressed
		viewer_.spinOnce();
	}
}

void ShowDebug::demo(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud)
{
	int v1 = 0;
	std::string  name01 = "sample01";
	viewer_.createViewPort(0.0, 0.0, 0.5, 1.0, v1);  //4个参数分别是X轴的最小值，最大值，Y轴的最小值，最大值，取值0-1，v1是标识
	viewer_.setBackgroundColor(0, 0, 0, v1);    //设置视口的背景颜色
	viewer_.addText("Radius: 0.01", 10, 10, "v1 text", v1);  //添加一个标签区别其他窗口  利用RGB颜色着色器并添加点云到视口中

	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> source_cloud_color_handler01(cloud, 0, 255, 0);
	viewer_.addPointCloud(cloud, source_cloud_color_handler01, name01, v1);

	pcl::PointCloud<pcl::PointXYZ>::Ptr transformCloud;
	Base00::ChapterDemo::transFormCloud(cloud, transformCloud);

	int v2 = 1;
	std::string  name02 = "sample02";
	viewer_.createViewPort(0.5, 0.0, 1.0, 1.0, v2);
	viewer_.setBackgroundColor(0.3, 0.3, 0.3, v2);
	viewer_.addText("Radius: 0.1", 10, 10, "v2 text", v2);

	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> source_cloud_color_handler02(transformCloud, 0, 0, 255);
	viewer_.addPointCloud(transformCloud, source_cloud_color_handler02, name02, v2);

	//为所有视口设置属性，
	viewer_.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, name01);
	viewer_.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, name02);
	viewer_.addCoordinateSystem(1.0);
}

