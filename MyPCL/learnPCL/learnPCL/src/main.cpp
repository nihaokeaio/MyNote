#include <pcl/visualization/cloud_viewer.h>
#include <iostream>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <vtkAutoInit.h>

#include <CreatePCL.h>
#include <Base00.hpp>
#include <ShowDebug.h>
#include <KDTreeBase.h>
#include "RandSampleConsensus.h"

VTK_MODULE_INIT(vtkInteractionStyle);
VTK_MODULE_INIT(vtkRenderingFreeType);
VTK_MODULE_INIT(vtkRenderingOpenGL2);

void foo()
{
    RandSampleConsensus randSampleConsensus;
    randSampleConsensus.demoSphere();
}


int main()
{
    foo();
    auto originCloudPtr = Chapter00::CreateCloud::createRandomPointCloud(500, 200, "test.pcd");

    pcl::PointCloud<pcl::PointXYZ>::Ptr readCloudPtr(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::io::loadPCDFile("test.pcd", *readCloudPtr);

    pcl::PointCloud<pcl::PointXYZ>::Ptr transformCloud;
    Base00::ChapterDemo::transFormCloud(readCloudPtr, transformCloud);

    ShowDebug showDebug;
    /*showDebug.addPoints(originCloudPtr, "sourceCloud");
    showDebug.addPoints(transformCloud, "transformCloud");*/
    showDebug.demo(originCloudPtr);

    /*KDTreeBase kdTreeBase;
    kdTreeBase.demoQuery();*/

    showDebug.run();
    
    return 0;
}