#include <BVHBuild.h>
#include <Triangle.h>
#include <global.hpp>
#include <Timer.h>

BVHBuild::BVHBuild(TriangleMesh* mesh)
{
	std::vector<std::shared_ptr<Object>> obj(mesh->triangles.size(), nullptr);
	/*for (const auto& t : mesh->triangles)
	{
		obj.push_back(t);
	}*/
	std::transform(mesh->triangles.begin(), mesh->triangles.end(), obj.begin(), [](const std::unique_ptr<Triangle>& t)
		{
			return std::shared_ptr<Object>(t.get());
		});
	Timer timer;
	root = buildBVH(obj);
	float ms = timer.stop();
	std::cout << "BVH Generation complete: \nTime Taken: " << ms << " microsecond \n\n";
}


BVHBuildNode* BVHBuild::buildBVH(std::vector<std::shared_ptr<Object>> triangles)
{
	Timer timer;
	root = recursiveBuild(triangles);
	float ms = timer.stop();
	std::cout << "BVH Generation complete: \nTime Taken: " << ms << " microsecond \n\n";
	return root;
}

Intersection BVHBuild::intersect(const Ray& ray)
{
	Intersection intersect;
	if (!root)
		return intersect;
	intersect = getIntersect(root, ray);
	if(intersect.happened)
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

void BVHBuild::getSample(BVHBuildNode* node, float p, Intersection& ints, float& pdf)
{
	if (node->left == nullptr || node->right == nullptr)
	{
		node->obj->sample(ints, pdf);
		pdf *= node->obj->getArea();
		return;
	}
	if (p < node->left->area)
	{
		getSample(node->left, p, ints, pdf);
	}
	else
	{
		getSample(node->right, p, ints, pdf);
	}
}

void BVHBuild::sample(Intersection& ints, float& pdf)
{
	float p = GamesMath::getRandomFloat() * root->area;
	getSample(root, p, ints, pdf);
	pdf /= root->area;	
}

BVHBuildNode* BVHBuild::recursiveBuild(std::vector<std::shared_ptr<Object>> objects)
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
				std::sort(objects.begin(), objects.end(), [](const std::shared_ptr<Object>& obj1, const std::shared_ptr<Object>& obj2) {
					return obj1->getBounds().Centroid().x > obj2->getBounds().Centroid().x;
					});
				break;
			}
			case Bounds3::MaxDim::YDim:
			{
				std::sort(objects.begin(), objects.end(), [](const std::shared_ptr<Object>& obj1, const std::shared_ptr<Object>& obj2) {
					return obj1->getBounds().Centroid().y > obj2->getBounds().Centroid().y;
					});
				break;
			}
			default:
			{
				std::sort(objects.begin(), objects.end(), [](const std::shared_ptr<Object>& obj1, const std::shared_ptr<Object>& obj2) {
					return obj1->getBounds().Centroid().z > obj2->getBounds().Centroid().z;
					});
				break;
			}
		}
		auto midObjIndex = objects.size() / 2;

		auto leftObjects = std::vector<std::shared_ptr<Object>>(objects.begin(), objects.begin() + midObjIndex);
		auto rightObjects = std::vector<std::shared_ptr<Object>>(objects.begin() + midObjIndex, objects.end());
		assert(objects.size() == (leftObjects.size() + rightObjects.size()));

		node->left = recursiveBuild(leftObjects);
		node->right = recursiveBuild(rightObjects);
		node->bound = node->left->bound + node->right->bound;
		node->area = node->left->area + node->right->area;
	}
	return node;
}
