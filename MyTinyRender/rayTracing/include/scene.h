#pragma once
#include <memory>
#include <Vector.hpp>
#include <Ray.h>
#include <Render.h>


class Object;
struct Light;


class Scene
{
public:
	Scene() = default;
	Scene(int width, int height);


	enum MASSTYPE
	{
		MSAA_4=4,
		MSAA_9=9,
		MSAA_16=16
	};

public:
	void render();
	void setRenderWay(Render::RenderWay way);

	void rayTracing();
	Vec3f castRay(Ray ray);

	void add(const std::shared_ptr<Object>& obj);
	void add(const std::shared_ptr<Light>& light);

	//得到索引
	int getIndex(int x, int y) const;
	int getMsaaIndex(int h, int v) const;
	//缓存数据
	void setDataBuffer(int x, int y, const Vec3f& data);

	//深度值
	void setZBuffer(int x, int y, const float val);
	float getZBuffer(int x, int y) const;

private:
	Render::RenderWay way_;

	int width_;
	int height_;
	float fov_ = 45;

	int Mass_;

	std::vector<std::vector<Vec3f>>frameBuffer_;
	std::vector<std::vector<float>>zBuffer_;

	std::vector<std::shared_ptr<Object>> objects_;
	std::vector<std::shared_ptr<Light> > lights_;

};
