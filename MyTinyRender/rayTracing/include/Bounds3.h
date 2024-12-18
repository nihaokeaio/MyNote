#pragma once
#include <Vector.hpp>
#include <Ray.h>

class Bounds3
{
public:

	enum MaxDim
	{
		XDim = 0,
		YDim = 1,
		ZDim = 2
	};

	Bounds3()
	{
		constexpr float minNum = std::numeric_limits<float>::lowest();
		constexpr float maxNum = std::numeric_limits<float>::max();
		pMax = Vec3f(minNum, minNum, minNum);
		pMin = Vec3f(maxNum, maxNum, maxNum);
	}

	Bounds3(Vec3f p)
	{
		pMax = p;
		pMin = p;
	}

	Bounds3(const Vec3f& min, const Vec3f& max)
	{
		pMax = max;
		pMin = min;
	}

	Bounds3 operator+(const Bounds3& b) const
	{
		Vec3f bMin = Vec3f(std::min(pMin.x, b.pMin.x), std::min(pMin.y, b.pMin.y), std::min(pMin.z, b.pMin.z));
		Vec3f bMax = Vec3f(std::max(pMax.x, b.pMax.x), std::max(pMax.y, b.pMax.y), std::max(pMax.z, b.pMax.z));
		return Bounds3(bMin, bMax);
	}

	Vec3f Centroid() { return 0.5 * pMin + 0.5 * pMax; }

	//并集
	Bounds3 boundsUnion(const Bounds3& a, const Bounds3& b)
	{
		Vec3f min = Vec3f(std::max(pMin.x, b.pMin.x), std::max(pMin.y, b.pMin.y), std::max(pMin.z, b.pMin.z));
		Vec3f max = Vec3f(std::min(pMax.x, b.pMax.x), std::min(pMax.y, b.pMax.y), std::min(pMax.z, b.pMax.z));
		return Bounds3(min, max);
	}

	//交集
	Bounds3 overlap(const Bounds3& a, const Bounds3& b)
	{
		return a + b;
	}

	MaxDim getMaxDim()
	{
		Vec3f d = pMax - pMin;
		if (d.x > d.y && d.x > d.z)
		{
			return XDim;
		}
		else if (d.y > d.x)
		{
			return YDim;
		}
		else
			return ZDim;
	}

	///是否与包围盒相交
	bool interset(const Ray& ray) const
	{
		float txMin = (pMin.x - ray.ori.x) / ray.dir.x;
		float txMax = (pMax.x - ray.ori.x) / ray.dir.x;
		float tyMin = (pMin.y - ray.ori.y) / ray.dir.y;
		float tyMax = (pMax.y - ray.ori.y) / ray.dir.y;
		float tzMin = (pMin.z - ray.ori.z) / ray.dir.z;
		float tzMax = (pMax.z - ray.ori.z) / ray.dir.z;

		if (txMin == txMax)
		{
			txMin = 0;
			txMax = std::numeric_limits<float>::infinity();
		}
		if (tyMin == tyMax)
		{
			tyMin = 0;
			tyMax = std::numeric_limits<float>::infinity();
		}
		if (tzMin == tzMax)
		{
			tzMin = 0;
			tzMax = std::numeric_limits<float>::infinity();
		}

		if (txMin > txMax)std::swap(txMin, txMax);
		if (tyMin > tyMax)std::swap(tyMin, tyMax);
		if (tzMin > tzMax)std::swap(tzMin, tzMax);

		float tEnter = std::max(std::max(txMin, tyMin), tzMin);
		float tExit = std::min(std::min(txMax, tyMax), tzMax);
		
		if (tEnter < tExit && tExit >= 0)
		{
			return true;
		}
		return false;

	}

public:
	Vec3f pMax;
	Vec3f pMin;
};