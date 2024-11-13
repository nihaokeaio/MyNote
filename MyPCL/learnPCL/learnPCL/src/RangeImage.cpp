#include "RangeImage.h"
#include <pcl/range_image/range_image.h>
#include "ShowDebug.h"

RangeImage::RangeImage()
{
    viewer_.reset(new ShowDebug("RangeImage"));
}

void RangeImage::demo()
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr readCloudPtr(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::io::loadPCDFile("test.pcd", *readCloudPtr);

    auto rangeCloud = createRangeImage(readCloudPtr);

    viewer_->addPoints(readCloudPtr, "originPCL", Eigen::Vector4f(0.0f, 0.0f, 1.0f, 1.0));

    viewer_->addPoints(rangeCloud, "rangePCL", Eigen::Vector4f(0.0f, 1.0f, 0.0f, 1.0));

    viewer_->run();
}

std::shared_ptr<pcl::RangeImage> RangeImage::createRangeImage(const pcl::PointCloud<pcl::PointXYZ>::Ptr& pCloud)
{
    /*关于range_image.createFromPointCloud（）参数的解释 （涉及的角度都为弧度为单位);
    point_cloud为创建深度图像所需要的点云
    angular_resolution_x深度传感器X方向的角度分辨率
    angular_resolution_y深度传感器Y方向的角度分辨率
    pcl::deg2rad(360.0f)深度传感器的水平最大采样角度
    pcl::deg2rad(180.0f)垂直最大采样角度
    scene_sensor_pose设置的模拟传感器的位姿是一个仿射变换矩阵，默认为4 * 4的单位矩阵变换
    coordinate_frame定义按照那种坐标系统的习惯  默认为CAMERA_FRAME
    noise_level  获取深度图像深度时，邻近点对查询点距离值的影响水平
    min_range 设置最小的获取距离，小于最小的获取距离的位置为传感器的盲区
    border_size  设置获取深度图像边缘的宽度 默认为0*/

    float angleResolution = pcl::deg2rad(1.0);
    float maxAngleWidth = pcl::deg2rad(360.f);  
    float maxAngleHeight = pcl::deg2rad(180.f);

    Eigen::Affine3f sensorPose = (Eigen::Affine3f)Eigen::Translation3f(0.0f, 0.0f, 0.0f);
    pcl::RangeImage::CoordinateFrame coordinateFrame = pcl::RangeImage::CAMERA_FRAME;
    float noiseLevel = 0.0f;
    float minRange = 0.0f;
    int borderSize = 1;

    std::shared_ptr<pcl::RangeImage> rangeImage(new pcl::RangeImage);
    rangeImage->createFromPointCloud(*pCloud, angleResolution, maxAngleWidth, maxAngleHeight, sensorPose, coordinateFrame,
        noiseLevel, minRange, borderSize);

    std::cout << rangeImage << std::endl;
    
    return rangeImage;
}
