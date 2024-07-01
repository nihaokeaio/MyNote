#pragma once
#include "Mesh.h"
#include "Texture.h"

struct fragment_shader_payload
{
	fragment_shader_payload();


	explicit fragment_shader_payload(const Vec3f& col, const Vec3f& nor, const Vec2f& tc, const Mesh* m) :
		color(col), normal(nor), tex_coords(tc), mesh(m)
	{
	}

	Vec3f view_pos;
	Vec3f color;
	Vec3f normal;
	Vec2f tex_coords;
	const Mesh* mesh = nullptr;
};

struct vertex_shader_payload
{
	Eigen::Vector3f position;
};
