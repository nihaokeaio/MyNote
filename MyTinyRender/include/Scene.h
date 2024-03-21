#pragma once
#include <vector>

#include "global.hpp"
#include "Vector.hpp"


class Geometry;


class Scene
{
public:
	int width_ = 1280;
	int height_ = 960;
	std::vector<Vec3f>data_buffer_;
	std::vector<float>z_buffer_;
	std::vector<std::vector<float>>z_buffer_msaa_;
	bool use_msaa_ = false;
	int msaa_spp = 4;
public:
	Scene();

	void addModel(std::vector<Geometry*> geometries);

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