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
    /*����range_image.createFromPointCloud���������Ľ��� ���漰�ĽǶȶ�Ϊ����Ϊ��λ);
    point_cloudΪ�������ͼ������Ҫ�ĵ���
    angular_resolution_x��ȴ�����X����ĽǶȷֱ���
    angular_resolution_y��ȴ�����Y����ĽǶȷֱ���
    pcl::deg2rad(360.0f)��ȴ�������ˮƽ�������Ƕ�
    pcl::deg2rad(180.0f)��ֱ�������Ƕ�
    scene_sensor_pose���õ�ģ�⴫������λ����һ������任����Ĭ��Ϊ4 * 4�ĵ�λ����任
    coordinate_frame���尴����������ϵͳ��ϰ��  Ĭ��ΪCAMERA_FRAME
    noise_level  ��ȡ���ͼ�����ʱ���ڽ���Բ�ѯ�����ֵ��Ӱ��ˮƽ
    min_range ������С�Ļ�ȡ���룬С����С�Ļ�ȡ�����λ��Ϊ��������ä��
    border_size  ���û�ȡ���ͼ���Ե�Ŀ�� Ĭ��Ϊ0*/

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
