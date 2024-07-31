#pragma once
#include <memory>
#include <vector>


class Scene;

class Render
{
public:
	enum class RenderWay
	{
		Rasterizer = 0,
		RayTracing =1
	};

public:
	Render() = default;

	void addScene(Scene* scene, RenderWay way = RenderWay::Rasterizer);

	void render();
public:

	std::vector<Scene*>scenes;
};
