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

	std::vector<int> inliers;  //�洢���ڵ㼯�ϵĵ������������
	{
		pcl::RandomSampleConsensus<pcl::PointXYZ> ransac(modelSphere);
		ransac.setDistanceThreshold(0.01);
		ransac.computeModel();
		ransac.getInliers(inliers);
	}

	///��ֵ���ڵ�
	pcl::PointCloud<pcl::PointXYZ>::Ptr outCloud((new pcl::PointCloud<pcl::PointXYZ>));
	pcl::copyPointCloud(*sphereCloud, inliers, *outCloud);

	viewer_->addPoints(outCloud, "sampleCSphere", MyColor::Red);

	pcl::PointCloud<pcl::PointXYZ>::Ptr transformCloud;
	Base00::ChapterDemo::transFormCloud(outCloud, transformCloud);

	viewer_->addPoints(transformCloud, "TransSphere", MyColor::Blue);

	viewer_->run();
}

void RandSampleConsensus::demoPlane()
{
	Eigen::Vector3f center(0.5, 0.5, 0.5);
	Eigen::Vector3f normal(0, 1, 1);
	float radius = 5.0f;
	auto planeCloud = Chapter00::CreateCloud::createRandomPlanePointCloud(center, normal);
	viewer_->addPoints(planeCloud, "randomSphere", MyColor::Green);
	//viewer_->run();

	pcl::SampleConsensusModelPlane<pcl::PointXYZ>::Ptr modelPlane(new pcl::SampleConsensusModelPlane<pcl::PointXYZ>(planeCloud));

	std::vector<int> inliers;  //�洢���ڵ㼯�ϵĵ������������
	{
		pcl::RandomSampleConsensus<pcl::PointXYZ> ransac(modelPlane);
		ransac.setDistanceThreshold(0.01);
		ransac.computeModel();
		ransac.getInliers(inliers);
	}

	///��ֵ���ڵ�
	pcl::PointCloud<pcl::PointXYZ>::Ptr outCloud((new pcl::PointCloud<pcl::PointXYZ>));
	pcl::copyPointCloud(*planeCloud, inliers, *outCloud);

	viewer_->addPoints(outCloud, "samplePlane", MyColor::Red);

	pcl::PointCloud<pcl::PointXYZ>::Ptr transformCloud;
	Base00::ChapterDemo::transFormCloud(outCloud, transformCloud);

	viewer_->addPoints(transformCloud, "TransPlane", MyColor::Blue);

	viewer_->run();
}
