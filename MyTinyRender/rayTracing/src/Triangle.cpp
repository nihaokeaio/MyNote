#include "Triangle.h"
#include <LoaderMesh.h>

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
	Vec3f qvec = tvec.dot(e1);
	v = ray.dir.dot(qvec) * det_inv;
	if (v < 0 || u + v > 1)
		return inter;
	t_tmp = e2.dot(qvec) * det_inv;

	inter.happened = true;
	inter.intsCoords = ray.ori + ray.dir * t_tmp;
	inter.normal = normal.normalize();
	inter.m = this->m;
	inter.object = this;
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
	return Bounds3(points[0], points[1]) + Bounds3(points[2]);
}

TriangleMesh::TriangleMesh(const std::string& fileName, Material* material)
{
	m = material;
	LoaderMesh load;
	load.loadObjFile(fileName);
	assert(load.LoadedMeshes_.size() == 1);

	const auto& mesh = load.LoadedMeshes_[0];

	vertexIndexs = mesh->indices_;

	int numVertex = mesh->vertices_.size();

	Vec3f bMin(std::numeric_limits<float>::max());
	Vec3f bMax(-std::numeric_limits<float>::max());

	for (int i = 0; i < numVertex; i += 3)
	{
		std::array<Vec3f, 0>vets;
		std::vector<Vec2f>sts(3, 0);
		for (int j = 0; j < 3; ++j)
		{
			auto pos = mesh->vertices_[i + j].position_;
			auto st = mesh->vertices_[i + j].textureCoordinate_;
			vets[i] = pos;
			sts[i] = st;
			bMin = Vec3f(std::min(bMin.x, pos.x), std::min(bMin.y, pos.y), std::min(bMin.z, pos.z));
			bMax = Vec3f(std::max(bMax.x, pos.x), std::max(bMax.y, pos.y), std::max(bMax.z, pos.z));
		}
		Triangle triangle(vets[0], vets[1], vets[2]);
		triangle.setCoordTextures(sts);
		triangles.push_back(&triangle);
	}
	bounds3 = Bounds3(bMin, bMax);
}

Intersection TriangleMesh::intersect(const Ray& ray)
{
	Intersection intersect;
	//to do
	for (auto& triangle : triangles)
	{
		const auto& ret = triangle->intersect(ray);
		if (ret.distance < intersect.distance)
		{
			intersect = ret;
		}
	}

	return intersect;
}

void TriangleMesh::getSurfaceProperties(const Vec3f& P, const Vec3f& I, const uint32_t& index, const Vec2f& uv, Vec3f& N, Vec2f& st) const
{
	Triangle* triangle = triangles[index];
	
	N = triangle->normal;

	const Vec2f& st0 = triangle->coordTextures[0];
	const Vec2f& st1 = triangle->coordTextures[1];
	const Vec2f& st2 = triangle->coordTextures[2];
	st = st0 * (1 - uv.x - uv.y) + st1 * uv.x + st2 * uv.y;
}

Vec3f TriangleMesh::evalDiffuseColor(const Vec2f& st) const
{
	float scale = 5;
	float pattern =
		(fmodf(st.x * scale, 1) > 0.5) ^ (fmodf(st.y * scale, 1) > 0.5);
	return Vec3f::lerp(Vec3f(0.815, 0.235, 0.031),
		Vec3f(0.937, 0.937, 0.231), pattern);
}

Bounds3 TriangleMesh::getBounds()
{
	return bounds3;
}
