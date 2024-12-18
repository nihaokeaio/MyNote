#include "Render.h"
#include "scene.h"

void Render::addScene(Scene* scene, RenderWay way)
{
	scene->setRenderWay(way);
	scenes.push_back(scene);
}

void Render::render()
{
	for (const auto& s : scenes)
	{
		s->render();
	}
}
