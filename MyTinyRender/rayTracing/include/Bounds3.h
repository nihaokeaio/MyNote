#pragma once
#include <Vector.hpp>
#include <Ray.h>

class Bounds3
{
public:
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

	//����
	Bounds3 boundsUnion(const Bounds3& a, const Bounds3& b)
	{
		Vec3f min = Vec3f(std::max(pMin.x, b.pMin.x), std::max(pMin.y, b.pMin.y), std::max(pMin.z, b.pMin.z));
		Vec3f max = Vec3f(std::min(pMax.x, b.pMax.x), std::min(pMax.y, b.pMax.y), std::min(pMax.z, b.pMax.z));
		return Bounds3(min, max);
	}

	//����
	Bounds3 overlap(const Bounds3& a, const Bounds3& b)
	{
		return a + b;
	}

	///�Ƿ����Χ���ཻ
	bool interset(Ray ray)
	{
		float txMin = (pMin.x - ray.ori.x) / ray.dir.x;
		float txMax = (pMin.x - ray.ori.x) / ray.dir.x;
		float tyMin = (pMin.y - ray.ori.y) / ray.dir.y;
		float tyMax = (pMin.y - ray.ori.y) / ray.dir.y;
		float tzMin = (pMin.z - ray.ori.z) / ray.dir.z;
		float tzMax = (pMin.z - ray.ori.z) / ray.dir.z;

		if (txMin > txMax)std::swap(txMax, txMax);
		if (tyMin > tyMax)std::swap(tyMin, tyMax);
		if (tzMin > tzMax)std::swap(tzMin, tzMax);

		float tEnter = fmax(fmax(txMin, tyMin), tzMin);
		float tExit = fmin(fmin(txMax, tyMax), tzMax);
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