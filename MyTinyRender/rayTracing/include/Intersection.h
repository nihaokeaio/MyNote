#pragma once
#include <Vector.hpp>

class Object;
class Material;


class Intersection
{
public:
	Intersection()
	{
		happened = false;
		intsCoords = Vec3f();
		normal = Vec3f();
		index = 0;
		distance = std::numeric_limits<double>::max();
		object = nullptr;
		m = nullptr;
	}

	Object* object;
	double distance;
	bool happened;

	Vec3f intsCoords;
	Vec3f normal;
	Vec3f emitColor;
	uint index;
	Vec2f st;
	std::shared_ptr<Material> m;
};