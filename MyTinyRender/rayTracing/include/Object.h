#pragma once
#include <Bounds3.h>
#include <Intersection.h>

class Object
{
public:
	virtual Intersection intersect(const Ray& ray){ return Intersection(); };
	virtual void getSurfaceProperties(const Vec3f& P, const Vec3f& I, const uint32_t& index, const Vec2f& uv, Vec3f& N, Vec2f& st) const = 0;
	virtual Vec3f evalDiffuseColor(const Vec2f&) const = 0;
	virtual Bounds3 getBounds() = 0;
	virtual float getArea() = 0;
	virtual ~Object() = default;

	int id = 0;
};


struct Light
{
	Light(const Vec3f& p, const Vec3f& I) :position(p), emissionColor(I) {}
	Vec3f position;
	Vec3f emissionColor;
};
