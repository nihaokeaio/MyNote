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
	std::shared_ptr<Object> obj;
	Bounds3 bound;
	float area;
};

class BVHBuild
{
public:
	BVHBuild() = default;
	
	BVHBuild(TriangleMesh* mesh);


	BVHBuildNode* buildBVH(std::vector<std::shared_ptr<Object>> triangles);

	Intersection intersect(const Ray& ray);

	Intersection getIntersect(const BVHBuildNode* node ,const Ray& ray);

	void getSample(BVHBuildNode* node, float p, Intersection& ints, float& pdf);

	void sample(Intersection& ints, float& pdf);
private:
	BVHBuildNode* root;

private:
	// BVHAccel Private Methods
	BVHBuildNode* recursiveBuild(std::vector<std::shared_ptr<Object>> objects);
};
