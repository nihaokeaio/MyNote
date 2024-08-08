#pragma once
#include <Object.h>


class Sphere :public Object
{
public:
	Vec3f center;
	float ridus;
	Material* m;
public:
	Intersection intersect(const Ray& ray);
	bool intersect(const Ray& ray, float& tnear, uint32_t& index);
	void getSurfaceProperties(const Vec3f& P, const Vec3f& I, const uint32_t& index, const Vec2f& uv, Vec3f& N, Vec2f& st)const override;
	Vec3f evalDiffuseColor(const Vec2f&) const override;
	Bounds3 getBounds()override;
};
