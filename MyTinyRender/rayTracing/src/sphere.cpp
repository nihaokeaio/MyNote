#include "sphere.h"
#include "Material.h"
#include "global.hpp"

Intersection Sphere::intersect(const Ray& ray)
{
    Intersection result;
    result.happened = false;
    Vec3f L = ray.ori - center;
    float a = ray.dir.norm2();
    float b = 2 * ray.dir.dot(L);
    float c = L.norm2() - ridus * ridus;
    float t0, t1;
    if (!GamesMath::solveQuadratic(a, b, c, t0, t1)) return result;
    if (t0 < 0) t0 = t1;
    if (t0 < 0) return result;
    result.happened = true;

    result.intsCoords = Vec3f(ray.ori + ray.dir * t0);
    result.normal = (result.intsCoords - center).normalize();
    result.m = this->m;
    result.object = this;
    result.distance = t0;
    result.index = 0;
    return result;
}

bool Sphere::intersect(const Ray& ray, float& tnear, uint32_t& index)
{
    const auto& ints = intersect(ray);
    tnear = ints.distance;
	return ints.happened;
}

void Sphere::getSurfaceProperties(const Vec3f& P, const Vec3f& I, const uint32_t& index, const Vec2f& uv, Vec3f& N, Vec2f& st) const
{
	N = (P - center).normalize();
}

Vec3f Sphere::evalDiffuseColor(const Vec2f&) const
{
    return m->m_color;
}

Bounds3 Sphere::getBounds()
{
	Vec3f pMax = center + ridus;
	Vec3f pMin = center - ridus;
	return Bounds3(pMin, pMax);
}

float Sphere::getArea()
{
    return 4 * M_PI * ridus * ridus;
}
