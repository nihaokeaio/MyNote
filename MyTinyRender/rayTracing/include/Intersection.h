#pragma once
#include <Vector.hpp>

class Object;
struct Material;


class Intersection
{
public:
	Intersection()
	{
		happened = false;
		intsCoords = Vec3f();
		normal = Vec3f();
		distance = std::numeric_limits<double>::max();
		object = nullptr;
		m = nullptr;
	}

	Object* object;
	double distance;
	bool happened;

	Vec3f intsCoords;
	Vec3f normal;
	Material* m;
};