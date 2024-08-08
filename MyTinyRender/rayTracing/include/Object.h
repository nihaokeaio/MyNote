#pragma once
#include <Ray.h>
#include <Intersection.h>
#include <Vector.hpp>
#include <Bounds3.h>

class Object
{
public:
	virtual Intersection intersect(const Ray& ray){ return Intersection(); };
	virtual void getSurfaceProperties(const Vec3f& P, const Vec3f& I, const uint32_t& index, const Vec2f& uv, Vec3f& N, Vec2f& st) const = 0;
	virtual Vec3f evalDiffuseColor(const Vec2f&) const = 0;
	virtual Bounds3 getBounds() = 0;
	virtual ~Object() = default;
};


struct Light
{
	Light(const Vec3f& p, const Vec3f& I) :position(p), intensity(I) {}
	Vec3f position;
	Vec3f intensity;
};
