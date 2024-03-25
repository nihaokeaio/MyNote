#pragma once
#include <vector>

#include "global.hpp"
#include "Vector.hpp"
#include <Eigen/Eigen>

enum class RasterizerWay
{
	DEFAULT = 0,
	BARYCENTRIC = 1,
	MASS = 2
};


class Geometry;


class Scene
{
public:
	int width_ = 1280;
	int height_ = 960;

	float near = 0.1f;
	float far = 50.f;
	float aspect_ratio = width_ / height_;
	float eye_fov = 45;

	std::vector<Vec3f>data_buffer_;
	std::vector<float>z_buffer_;
	std::vector<std::vector<float>>z_buffer_msaa_;
	bool use_msaa_ = false;
	int msaa_spp = 4;
public:
	Scene();

	void addModel(std::vector<Geometry*> geometries);

	void rasterizeTriangle(Geometry* geometry);


	Eigen::Matrix4f projectMatrix() const;

	void viewportMatrix(std::vector<Eigen::Vector4f>& vec);

	static Eigen::Matrix4f setViewMatrix(Vec3f eyePos);
	


	int get_index(int x,int y) const;
	//缓存数据
	void set_data_buffer(int x, int y, const Vec3f& data);
	void set_data_buffer(int index, const Vec3f& data);
	Vec3f get_data_buffer(int index) const;
	Vec3f get_data_buffer(int x, int y) const;
	//深度值
	void set_z_buffer(int index, const float val);
	void set_z_buffer(int x, int y, const float val);
	float get_z_buffer(int index) const;
	float get_z_buffer(int x, int y) const;

	void write() const;
	void useMsaa();
};