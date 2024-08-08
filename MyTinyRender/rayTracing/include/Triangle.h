#pragma once

#include <Vector.hpp>
#include <Object.h>
#include <Ray.h>

struct Vertex;
struct Material;

class Triangle :public Object
{
public:
	Vec3f points[3];		//逆时针方向为正
	Vec3f normal;
	Vec3f colors[3];
	Vec2f coordTextures[3];	
	Material* m;

public:

	Triangle(const Vec3f& v0, const Vec3f& v1, const Vec3f& v2, Material* material = nullptr)
	{
		points[0] = v0;
		points[1] = v1;
		points[2] = v2;
		m = material;
		normal = (v1 - v0).cross(v2 - v0).normalize();
	}

	void setCoordTextures(const std::vector<Vec2f>& st)
	{
		assert(st.size() == 3);
		for (int i = 0; i < 3; ++i)
		{
			coordTextures[i] = st[i];
		}
	}

	Intersection intersect(const Ray& ray) override;
	void getSurfaceProperties(const Vec3f& P, const Vec3f& I, const uint32_t& index, const Vec2f& uv, Vec3f& N, Vec2f& st)const override;
	Vec3f evalDiffuseColor(const Vec2f&) const override;
	Bounds3 getBounds()override;
};

class TriangleMesh : public Object
{
public:
	Material* m = nullptr;
	std::vector<Triangle*>triangles;
	std::vector<Vertex*>vertexs;
	std::vector<uint>vertexIndexs;
	Bounds3 bounds3;

public:
	TriangleMesh(const std::string& fileName, Material* material = nullptr);

	TriangleMesh(const std::vector<Vertex*>&v, const std::vector<uint>&indexs)
	{
		vertexs = v;
		vertexIndexs = indexs;
	}

	Intersection intersect(const Ray& ray) override;
	void getSurfaceProperties(const Vec3f& P, const Vec3f& I, const uint32_t& index, const Vec2f& uv, Vec3f& N, Vec2f& st)const override;
	Vec3f evalDiffuseColor(const Vec2f& st) const override;
	Bounds3 getBounds()override;

};
