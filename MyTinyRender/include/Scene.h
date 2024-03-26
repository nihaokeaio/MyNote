#pragma once
#include <vector>

#include "global.hpp"
#include "Vector.hpp"
#include <Eigen/Eigen>


class Geometry;
class Triangle;


enum class RasterizerWay
{
	DEFAULT = 0,
	BARYCENTRIC = 1,
	MASS = 2
};




class Scene
{
private:
	int width_ = 1280;
	int height_ = 960;

	float near = 0.1f;
	float far = 50.f;
	float aspectRatio_ = width_ / height_;
	float eyeFov_ = 45;

	std::vector<Vec3f>dataBuffer_;
	std::vector<float>zBuffer_;
	std::vector<std::vector<float>>zBufferMsaa_;
	bool useMsaa_ = false;
	int msaaH_ = 2, msaaV_ = 2;
	int msaaSpp_ = msaaH_ * msaaV_;
public:
	Scene();

	void addModel(std::vector<Geometry*> geometries);

	void rasterizeTriangle(Geometry* geometry);
	void rasterizeTriangle(int i, int y, RasterizerWay way, Triangle* triangle);


	Eigen::Matrix4f projectMatrix() const;

	void viewportMatrix(std::vector<Eigen::Vector4f>& vec);

	static Eigen::Matrix4f setViewMatrix(Vec3f eyePos);
	


	int getIndex(int x,int y) const;
	int getMsaaIndex(int h,int v) const;
	//缓存数据
	void setDataBuffer(int x, int y, const Vec3f& data);
	void setDataBuffer(int index, const Vec3f& data);
	Vec3f getDataBuffer(int index) const;
	Vec3f getDataBuffer(int x, int y) const;
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