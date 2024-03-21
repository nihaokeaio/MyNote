#pragma once
#include "Vector.hpp"
#include "Scene.h"
#include "global.hpp"

class Scene;


class BoundBox
{
public:
	BoundBox()
	{
		pMin_ = Vec3f();
		pMax_ = Vec3f();
	}
	
	BoundBox(Vec3f pMin, Vec3f pMax):pMin_(pMin),pMax_(pMax)
	{
	}

	void construct(Vec3f pMin, Vec3f pMax)
	{
		pMin_ = pMin;
		pMax_ = pMax;
	}

	Vec3f min() const { return pMin_; }

	Vec3f max() const { return pMax_; }
private:
	Vec3f pMin_, pMax_;
};




class Geometry
{
public:
	virtual void draw(Scene* scene) const{};
};

class Line : public Geometry
{
public:
	Line(Vec3f p0, Vec3f p1)
	{
		point[0] = p0;
		point[1] = p1;
		color = Color::White;
	}

	Line(int x0, int y0, int x1, int y1)
	{
		point[0].x = x0;
		point[0].y = y0;
		point[0].z = 0;
		point[1].x = x1;
		point[1].y = y1;
		point[1].z = 0;
		color = Color::White;
	}

	Line(int x0, int y0, int z0, int x1, int y1, int z1)
	{
		point[0].x = x0;
		point[0].y = y0;
		point[0].z = z0;
		point[1].x = x1;
		point[1].y = y1;
		point[1].z = z1;
		color = Color::White;
	}
	void draw(Scene* scene) const override;

	void setColor(Vec3f c);

	static void line(Vec2i p0, Vec2i p1, Scene* scene, Vec3f color);
	static void line(int x0, int y0, int x1, int y1, Scene* scene, Vec3f color);
private:
	Vec3f point[2];
	Vec3f color;
};

class Triangle : public Geometry
{
public:
	Triangle(const Vec3f p0, const Vec3f p1, const Vec3f p2)
	{
		point_[0] = p0;
		point_[1] = p1;
		point_[2] = p2;

		color_[0] = Color::White;
		color_[1] = Color::White;
		color_[2] = Color::White;

		const float pxMin = std::min(point_[0].x, std::min(point_[1].x, point_[2].x));
		const float pyMin = std::min(point_[0].y, std::min(point_[1].y, point_[2].y));
		const float pzMin = std::min(point_[0].z, std::min(point_[1].z, point_[2].z));
		const float pxMax = std::max(point_[0].x, std::max(point_[1].x, point_[2].x));
		const float pyMax = std::max(point_[0].y, std::max(point_[1].y, point_[2].y));
		const float pzMax = std::max(point_[0].z, std::max(point_[1].z, point_[2].z));

		box_.construct(Vec3f(pxMin, pyMin, pzMin), Vec3f(pxMax, pyMax, pzMax));
	}

	Triangle(const Vec2i p0, const Vec2i p1, const Vec2i p2)
	{
		point_[0] = Vec3f(p0.x, p0.y, 0);
		point_[1] = Vec3f(p1.x, p1.y, 0);
		point_[2] = Vec3f(p2.x, p2.y, 0);

		color_[0] = Color::White;
		color_[1] = Color::White;
		color_[2] = Color::White;

		const float pxMin = std::min(point_[0].x, std::min(point_[1].x, point_[2].x));
		const float pyMin = std::min(point_[0].y, std::min(point_[1].y, point_[2].y));
		const float pzMin = std::min(point_[0].z, std::min(point_[1].z, point_[2].z));
		const float pxMax = std::max(point_[0].x, std::max(point_[1].x, point_[2].x));
		const float pyMax = std::max(point_[0].y, std::max(point_[1].y, point_[2].y));
		const float pzMax = std::max(point_[0].z, std::max(point_[1].z, point_[2].z));

		box_.construct(Vec3f(pxMin, pyMin, pzMin), Vec3f(pxMax, pyMax, pzMax));
	}

	void setColor(Vec3f c);
	void setColor(const Vec3f* c);

	void setFill(bool enable) { isFill_ = enable; }
	bool getFill() const { return isFill_; }

	void draw(Scene* scene) const override;

	void fill(Scene* scene);
private:
	Vec3f point_[3];
	Vec3f color_[3];

	BoundBox box_;

	bool isFill_;
};


