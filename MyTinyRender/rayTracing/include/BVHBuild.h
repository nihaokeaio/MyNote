#pragma once

#include <Bounds3.h>
#include <Intersection.h>

class Object;
class Triangle;
class TriangleMesh;

struct BVHBuildNode
{
	BVHBuildNode* left;
	BVHBuildNode* right;
	Object* obj;
	Bounds3 bound;
	float area;
};

class BVHBuild
{
public:
	BVHBuild() = default;
	
	BVHBuild(TriangleMesh* mesh);


	BVHBuildNode* buildBVH(std::vector<Object*> triangles);

	Intersection intersect(const Ray& ray);

	Intersection getIntersect(const BVHBuildNode* node ,const Ray& ray);

	BVHBuildNode* root;

private:
	// BVHAccel Private Methods
	BVHBuildNode* recursiveBuild(std::vector<Object*> objects);
};
