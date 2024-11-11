#include "RandSampleConsensus.h"
#include "ShowDebug.h"
#include "CreatePCL.h"
#include "Base00.hpp"
#include "pcl/sample_consensus/ransac.h"
#include "pcl/sample_consensus/sac_model_sphere.h"
#include "pcl/sample_consensus/sac_model_plane.h"

RandSampleConsensus::RandSampleConsensus()
{
	viewer_.reset(new ShowDebug("Viewer01"));
}

void RandSampleConsensus::demoSphere()
{
	Eigen::Vector3f center(0, 0, 0);
	float radius = 5.0f;
	auto sphereCloud = Chapter00::CreateCloud::createRandomSpherePointCloud(center, radius);
	viewer_->addPoints(sphereCloud, "randomSphere", MyColor::Green);
	//viewer_->run();

	pcl::SampleConsensusModelSphere<pcl::PointXYZ>::Ptr modelSphere(new pcl::SampleConsensusModelSphere<pcl::PointXYZ>(sphereCloud));

	std::vector<int> inliers;  //存储局内点集合的点的索引的向量
	{
		pcl::RandomSampleConsensus<pcl::PointXYZ> ransac(modelSphere);
		ransac.setDistanceThreshold(0.01);
		ransac.computeModel();
		ransac.getInliers(inliers);
	}


	///赋值局内点
	pcl::PointCloud<pcl::PointXYZ>::Ptr outCloud((new pcl::PointCloud<pcl::PointXYZ>));
	pcl::copyPointCloud(*sphereCloud, inliers, *outCloud);

	viewer_->addPoints(outCloud, "sampleCSphere", MyColor::Red);

	/*pcl::PointCloud<pcl::PointXYZ>::Ptr transformCloud;
	Base00::ChapterDemo::transFormCloud(outCloud, transformCloud);

	viewer_->addPoints(transformCloud, "TransSphere", MyColor::Blue);*/

	viewer_->run();
}

void RandSampleConsensus::demoPlane()
{
}
