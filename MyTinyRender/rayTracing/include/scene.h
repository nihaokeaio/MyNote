#pragma once
#include <memory>
#include <Intersection.h>
#include <Ray.h>
#include <Render.h>

class Object;
struct Light;
class BVHBuild;


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

	Vec3f castRay(const Ray& ray, int depth);


	Vec3f castRaySPP(const Ray& ray, int spp);

	Vec3f pathTracing(const Ray& ray);

	Intersection intersect(const Ray& ray);
	void buildBVH();

	void add(const std::shared_ptr<Object>& obj);
	void add(const std::shared_ptr<Light>& light);

	void sampleLight(Intersection& ints, float& pdf) const;
	float P_RR() const;

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
	int maxDepth_ = 5;
	Vec3f cameraPos_;
	Vec3f backgroundColor_{ 0.235294, 0.67451, 0.843137 };
	//Vec3f backgroundColor_{ 0, 0, 0 };

	std::vector<Vec3f>frameBuffer_;
	std::vector<float>zBuffer_;

	std::vector<std::shared_ptr<Object>> objects_;
	std::vector<std::shared_ptr<Light> > lights_;

	std::shared_ptr<BVHBuild> bvhNode_;

};
