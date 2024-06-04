#pragma once
#include <vector>

#include "global.hpp"
#include "Vector.hpp"
#include <Eigen/Eigen>


class Camera;
class Rasterizer;
struct Mesh;
class Geometry;
class Triangle;







class Scene
{
	friend void mouseCallBack(int event, int x, int y, int flags, void* param);
private:

	std::vector<Vec3f>dataBuffer_={};
	std::vector<float>zBuffer_={};
	std::vector<std::vector<float>>zBufferMsaa_={};
	bool useMsaa_ = false;

	std::shared_ptr<Rasterizer> rasterizer_;

	std::shared_ptr<Camera>camera_;

	std::map<std::shared_ptr<Mesh>, std::vector<Geometry*>>meshCords_;
public:
	float near_ = 0.1f;
	float far_ = 50.f;
	float aspectRatio_ = width_ / height_;
	float eyeFov_ = 45;

	int width_ = 960;
	int height_ = 960;


	int msaaH_ = 2, msaaV_ = 2;
	int msaaSpp_ = msaaH_ * msaaV_;


public:
	Scene();

	void addMesh(const std::shared_ptr<Mesh>& mesh);

	void doUpDate();

	void rasterizeTriangle(Geometry* geometry, const Mesh*  mesh);

	std::vector<Eigen::Vector4f> getViewPortPos(const std::vector<Vec3f>& trianglePos, const Mesh* mesh) const;

	void viewportMatrix(std::vector<Eigen::Vector4f>& vec) const;

	static Eigen::Matrix4f setViewMatrix(Vec3f eyePos);
	
	void clear();

	Camera* getCamera() const { return  camera_.get(); }

	int getIndex(int x,int y) const;
	int getMsaaIndex(int h,int v) const;
	//缓存数据
	void setDataBuffer(int x, int y, const Vec3f& data);
	void setDataBuffer(int index, const Vec3f& data);
	Vec3f getDataBuffer(int index) const;
	Vec3f getDataBuffer(int x, int y) const;
	std::vector<Vec3f>& getDataBuffer() { return dataBuffer_; };
	//深度值
	void setZBuffer(int index, const float val);
	void setZBuffer(int x, int y, const float val);
	float getZBuffer(int index) const;
	float getZBuffer(int x, int y) const;


	void setMassZBuffer(int x, int y, int h, int v, const float val);
	void setMassZBuffer(int index, int h, int v, const float val);

	float getMassZBuffer(int x, int y, int h, int v) const;
	float getMassZBuffer(int index, int h, int v) const;

	void write() const;
	void useMsaa(int sizeH = 2, int sizeV = 2);
};

