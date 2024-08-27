#include <BVHBuild.h>
#include <Triangle.h>
#include <Timer.h>

BVHBuild::BVHBuild(TriangleMesh* mesh)
{
	std::vector<Object*> obj(mesh->triangles.size(), nullptr);
	/*for (const auto& t : mesh->triangles)
	{
		obj.push_back(t);
	}*/
	std::transform(mesh->triangles.begin(), mesh->triangles.end(), obj.begin(), [](const std::unique_ptr<Triangle>& t)
		{
			return static_cast<Object*>(t.get());
		});
	Timer timer;
	root = buildBVH(obj);
	float ms = timer.stop();
	std::cout << "BVH Generation complete: \nTime Taken: " << ms << " microsecond \n\n";
}

BVHBuildNode* BVHBuild::buildBVH(std::vector<Object*> triangles)
{
	return recursiveBuild(triangles);
}

Intersection BVHBuild::intersect(const Ray& ray)
{
	Intersection intersect;
	if (!root)
		return intersect;
	intersect = getIntersect(root, ray);
	intersect.index = intersect.object->id;
	return intersect;
}

Intersection BVHBuild::getIntersect(const BVHBuildNode* node ,const Ray& ray)
{
	Intersection intersect;
	if (!node)
		return intersect;

	if (node->bound.interset(ray))
	{
		if (!node->left && !node->right)
		{
			return node->obj->intersect(ray);
		}
		Intersection leftIntersect = getIntersect(node->left, ray);
		Intersection rightIntersect = getIntersect(node->right, ray);
		return leftIntersect.distance < rightIntersect.distance ? leftIntersect : rightIntersect;
	}
	return intersect;
}

BVHBuildNode* BVHBuild::recursiveBuild(std::vector<Object*> objects)
{
	BVHBuildNode* node = new BVHBuildNode();

	if (objects.size() == 1)
	{
		node->bound = objects[0]->getBounds();
		node->obj = objects[0];
		node->left = nullptr;
		node->right = nullptr;
		node->area = objects[0]->getArea();
		return node;
	}
	else if (objects.size() == 2)
	{
		node->left = recursiveBuild(std::vector{ objects[0] });
		node->right = recursiveBuild(std::vector{ objects[1] });
		node->bound = node->left->bound + node->right->bound;
		node->area = node->left->area + node->right->area;
		return node;
	}
	else
	{
		Bounds3 b;
		for (const auto& obj : objects)
		{
			b = b + obj->getBounds();
		}
		switch (b.getMaxDim())
		{
			case Bounds3::MaxDim::XDim:
			{
				std::sort(objects.begin(), objects.end(), [](Object* obj1, Object* obj2) {
					return obj1->getBounds().Centroid().x > obj2->getBounds().Centroid().x;
					});
				break;
			}
			case Bounds3::MaxDim::YDim:
			{
				std::sort(objects.begin(), objects.end(), [](Object* obj1, Object* obj2) {
					return obj1->getBounds().Centroid().y > obj2->getBounds().Centroid().y;
					});
				break;
			}
			default:
			{
				std::sort(objects.begin(), objects.end(), [](Object* obj1, Object* obj2) {
					return obj1->getBounds().Centroid().z > obj2->getBounds().Centroid().z;
					});
				break;
			}
		}
		auto midObjIndex = objects.size() / 2;

		auto leftObjects = std::vector<Object*>(objects.begin(), objects.begin() + midObjIndex);
		auto rightObjects = std::vector<Object*>(objects.begin() + midObjIndex, objects.end());
		assert(objects.size() == (leftObjects.size() + rightObjects.size()));

		node->left = recursiveBuild(leftObjects);
		node->right = recursiveBuild(rightObjects);
		node->bound = node->left->bound + node->right->bound;
		node->area = node->left->area + node->right->area;
	}
	return node;
}
