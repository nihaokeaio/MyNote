#include "Triangle.h"
#include "Material.h"
#include <LoaderMesh.h>
#include <BVHBuild.h>

Intersection Triangle::intersect(const Ray& ray)
{
	Intersection inter;
	Vec3f e1 = points[1] - points[0];
	Vec3f e2 = points[2] - points[0];
	if (ray.dir.dot(normal) > 0)
		return inter;
	double u, v, t_tmp = 0;
	Vec3f pvec = ray.dir.cross(e2);
	double det = e1.dot(pvec);
	if (fabs(det) < EPSILON)
		return inter;

	double det_inv = 1. / det;
	Vec3f tvec = ray.ori - points[0];
	u = tvec.dot(pvec) * det_inv;
	if (u < 0 || u > 1)
		return inter;
	Vec3f qvec = tvec.cross(e1);
	v = ray.dir.dot(qvec) * det_inv;
	if (v < 0 || u + v > 1)
		return inter;
	t_tmp = e2.dot(qvec) * det_inv;

	inter.happened = true;
	inter.intsCoords = ray.ori + ray.dir * t_tmp;
	inter.normal = normal.normalize();
	inter.m = this->m.get();
	inter.object = this;
	inter.index = 0;
	inter.st = { (float)u,(float)v };
	inter.distance = t_tmp;

	return inter;
}

void Triangle::getSurfaceProperties(const Vec3f& P, const Vec3f& I, const uint32_t& index, const Vec2f& uv, Vec3f& N, Vec2f& st) const
{
	N = normal;
}

Vec3f Triangle::evalDiffuseColor(const Vec2f&) const
{
	return Vec3f(0.5, 0.5, 0.5);
}

Bounds3 Triangle::getBounds()
{
	return Bounds3(points[0]) + Bounds3(points[1]) + Bounds3(points[2]);
}

float Triangle::getArea()
{
	Vec3f e1 = points[1] - points[0];
	Vec3f e2 = points[2] - points[0];
	return e1.cross(e2).norm() * 0.5;
}

TriangleMesh::TriangleMesh(const std::string& fileName, std::shared_ptr<Material> material)
{
	if (!material)
	{
		material.reset(new Material);
	}
	m = material;
	LoaderMesh::LoaderMesh load;
	load.loadObjFile(fileName);
	assert(load.LoadedMeshes_.size() == 1);

	const auto& mesh = load.LoadedMeshes_[0];

	vertexIndexs = mesh->indices_;

	int numVertex = mesh->vertices_.size();

	Vec3f bMin(std::numeric_limits<float>::max());
	Vec3f bMax(-std::numeric_limits<float>::max());

	for (int i = 0; i < numVertex; i += 3)
	{
		std::vector<Vec3f>vets(3, 0);
		std::vector<Vec2f>sts(3, 0);
		for (int j = 0; j < 3; ++j)
		{
			auto pos = mesh->vertices_[i + j].position_ * 30.f;
			auto st = mesh->vertices_[i + j].textureCoordinate_;
			vets[j] = pos;
			sts[j] = st;
			bMin = Vec3f(std::min(bMin.x, pos.x), std::min(bMin.y, pos.y), std::min(bMin.z, pos.z));
			bMax = Vec3f(std::max(bMax.x, pos.x), std::max(bMax.y, pos.y), std::max(bMax.z, pos.z));
		}

		auto material = std::make_shared<Material>(Material::MaterialType::DIFFUSE_AND_GLOSSY,
			Vec3f(0.5, 0.5, 0.5), Vec3f(0, 0, 0));
		material->Kd = 0.6;
		material->Ks = 0.0;
		material->specularExponent = 32;

		std::unique_ptr<Triangle> triangle(new Triangle(vets[0], vets[1], vets[2], material));
		triangle->setCoordTextures(sts);
		triangle->id = triangles.size();
		triangles.emplace_back(std::move(triangle));
	}
	bounds3 = Bounds3(bMin, bMax);
	if (!bvhBuild)
	{
		bvhBuild = std::make_shared<BVHBuild>(this);
	}
}

TriangleMesh::TriangleMesh(const std::vector<Vec3f>& v, const std::vector<uint>& indexs, const std::vector<Vec2f>& st, std::shared_ptr<Material> material)
{
	for (int i = 0; i < indexs.size(); i += 3)
	{
		std::vector<Vec3f>vets(3, 0);
		std::vector<Vec2f>sts(3, 0);
		for (int j = 0; j < 3; ++j)
		{
			vets[j] = v[indexs[i + j]];
			sts[j] = st[indexs[i + j]];
		}

		if (!material)
		{
			material = std::make_shared<Material>(Material::MaterialType::DIFFUSE_AND_GLOSSY,
				Vec3f(0.5, 0.5, 0.5), Vec3f(0, 0, 0));
			material->Kd = 0.6;
			material->Ks = 0.0;
			material->specularExponent = 32;
		}
		this->m = material;
		std::unique_ptr<Triangle> triangle(new Triangle(vets[0], vets[1], vets[2], material));
		triangle->setCoordTextures(sts);
		triangles.emplace_back(std::move(triangle));
	}
}

Intersection TriangleMesh::intersect(const Ray& ray)
{
	Intersection intersect;
	
	if (bvhBuild)
	{
		intersectByBVH(ray, intersect);
	}
	else
	{
		intersectByOrders(ray, intersect);
	}
	return intersect;
}

void TriangleMesh::getSurfaceProperties(const Vec3f& P, const Vec3f& I, const uint32_t& index, const Vec2f& uv, Vec3f& N, Vec2f& st) const
{
	Triangle* triangle = triangles[index].get();
	
	N = triangle->normal;

	const Vec2f& st0 = triangle->coordTextures[0];
	const Vec2f& st1 = triangle->coordTextures[1];
	const Vec2f& st2 = triangle->coordTextures[2];
	st = st0 * (1 - uv.x - uv.y) + st1 * uv.x + st2 * uv.y;
}

Vec3f TriangleMesh::evalDiffuseColor(const Vec2f& st) const
{
	float scale = 10;
	float pattern =
		(fmodf(st.x * scale, 1) > 0.5) ^ (fmodf(st.y * scale, 1) > 0.5);
	return Vec3f::lerp(Vec3f(0.815, 0.235, 0.031),
		Vec3f(0.937, 0.937, 0.231), pattern);
}

Bounds3 TriangleMesh::getBounds()
{
	return bounds3;
}

float TriangleMesh::getArea()
{
	return area;
}

void TriangleMesh::intersectByOrders(const Ray& ray, Intersection& intersect)
{
	for (auto& triangle : triangles)
	{
		const auto& ret = triangle->intersect(ray);
		if (ret.distance < intersect.distance)
		{
			intersect = ret;
		}
	}
}

void TriangleMesh::intersectByBVH(const Ray& ray, Intersection& intersect)
{
	intersect =  bvhBuild->intersect(ray);
}
