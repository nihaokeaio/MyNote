#pragma once
#include "TriangleMesh.h"
#include "Texture.h"
#include "Camera.h"

struct fragment_shader_payload
{
	fragment_shader_payload();


	explicit fragment_shader_payload(const Vec3f& p, const Vec3f& col, const Vec3f& nor, const Vec2f& tc, const TriangleMesh* m) :
		point(p), color(col), normal(nor), tex_coords(tc), mesh(m)
	{
	}

	void setCamera(Camera* camera)
	{
		view_pos = camera->getCameraPos();
		view_dir = camera->getCameraDir();
	}

	Vec3f point;
	Vec3f view_pos;
	Vec3f view_dir;
	Vec3f color;
	Vec3f normal;
	Vec2f tex_coords;
	const TriangleMesh* mesh = nullptr;
};

struct vertex_shader_payload
{
	Vec3f position;
};
