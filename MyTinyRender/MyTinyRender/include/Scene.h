#pragma once
#include <vector>

#include "global.hpp"
#include "Vector.hpp"

#include <Rasterizer.h>
#include <Shader.h>


//struct Vertex;
class Camera;
struct TriangleMesh;
class Geometry;
class Triangle;







class Scene
{
	friend void mouseCallBack(int event, int x, int y, int flags, void* param);
private:

	Vec3Vector dataBuffer_={};
	std::vector<float>zBuffer_={};
	std::vector<std::vector<float>>zBufferMsaa_={};
	bool useMsaa_ = false;

	std::unique_ptr<Rasterizer> rasterizer_;

	std::shared_ptr<Camera>camera_;

	std::map<std::shared_ptr<TriangleMesh>, std::vector<Geometry*>>meshCords_;


public:
	int width_ = 1280;
	int height_ = 960;
	float near_ = 0.1f;
	float far_ = 50.f;
	float aspectRatio_ = (float)width_ / (float)height_;
	float eyeFov_ = 45;



	int msaaH_ = 2, msaaV_ = 2;
	int msaaSpp_ = msaaH_ * msaaV_;


public:
	Scene();

	void addMesh(const std::shared_ptr<TriangleMesh>& mesh);

	void doUpDate();

	void rasterizeTriangle(Geometry* geometry, const Vec3Vector& points, const TriangleMesh* mesh);

	void setFragmentShaderFunction(const std::function<Vec3f(fragment_shader_payload)>& fun) const noexcept;
	void setVertexShaderFunction(const std::function<Vec3f(vertex_shader_payload)>& fun) const noexcept;


	Vec4Vector getViewPortPos(const Vec3Vector& trianglePos, const Matrix4x4f& mat) const;
	///向量应该移除平移参数
	Vec3Vector getPointPos(const Vec3Vector& trianglePos, const Matrix4x4f& mat, bool isVec) const;
	Vec3Vector getVertexNormal(const Vec3Vector& vertexNormal, const Matrix4x4f& mat) const;

	void viewportMatrix(Vec4Vector& vec) const;
	Matrix4x4f getMVPMat(const TriangleMesh* mesh)const;
	Matrix4x4f getMVMat(const TriangleMesh* mesh) const;

	static Matrix4x4f setViewMatrix(Vec3f eyePos);
	
	void clear();

	Camera* getCamera() const { return  camera_.get(); }


	int getIndex(int x,int y) const;
	int getMsaaIndex(int h,int v) const;
	//缓存数据
	void setDataBuffer(int x, int y, const Vec3f& data);
	void setDataBuffer(int index, const Vec3f& data);
	Vec3f getDataBuffer(int index) const;
	Vec3f getDataBuffer(int x, int y) const;
	Vec3Vector& getDataBuffer() { return dataBuffer_; };
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

