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
#include "OcTreeBase.h"
#include "SimpleOpenNIViewer.h"
#include "RangeImage.h"
#include "FilterBase.h"

VTK_MODULE_INIT(vtkInteractionStyle);
VTK_MODULE_INIT(vtkRenderingFreeType);
VTK_MODULE_INIT(vtkRenderingOpenGL2);

void foo()
{
    //采样一致性
    if(false)
    {
        RandSampleConsensus randSampleConsensus;
        randSampleConsensus.demoPlane();
    }

    //深度图
    if (false)
    {
        RangeImage rangeImage;
        rangeImage.demo();
    }

    if (true)
    {
        FilterBase filterBase;
        filterBase.demo();
    }
    
}


int main()
{
    foo();
    auto originCloudPtr = Chapter00::CreateCloud::createRandomPointCloud(500, 200, "test.pcd");

    pcl::PointCloud<pcl::PointXYZ>::Ptr readCloudPtr(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::io::loadPCDFile("test.pcd", *readCloudPtr);

    pcl::PointCloud<pcl::PointXYZ>::Ptr transformCloud;
    Base00::ChapterDemo::transFormCloud(readCloudPtr, transformCloud);

    {
        //OcTree::ocTreeChangeDetection();
    }

    {
       /* SimpleOpenNIViewer simpleOpenNIViewer;
        simpleOpenNIViewer.run();*/
    }


    ShowDebug showDebug("DoubleSceen");
    /*showDebug.addPoints(originCloudPtr, "sourceCloud");
    showDebug.addPoints(transformCloud, "transformCloud");*/
    showDebug.demo(originCloudPtr);

    /*KDTreeBase kdTreeBase;
    kdTreeBase.demoQuery();*/

    showDebug.run();
    
    return 0;
}