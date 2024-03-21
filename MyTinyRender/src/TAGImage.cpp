#include "Scene.h"
#include "global.hpp"
#include "Geometry.h"

Scene::Scene()
{
	data_buffer_.resize(width_ * height_);
	z_buffer_.resize(width_ * height_);
	std::fill(z_buffer_.begin(), z_buffer_.end(), std::numeric_limits<float>::infinity());
}

void Scene::addModel(const std::vector<Geometry*> geometries)
{
	for(const auto& model:geometries)
	{
		//投影变换

		//光栅化
		model->draw(this);
	}
}

int Scene::get_index(int x, int y) const
{
	return (height_ - y) * width_ + x;
}

Vec3f Scene::get_data_buffer(int index) const
{
	return data_buffer_[index];
}

Vec3f Scene::get_data_buffer(int x, int y) const
{
	return data_buffer_[get_index(x, y)];
}

void Scene::set_data_buffer(int x, int y, const Vec3f& data)
{
	data_buffer_[get_index(x, y)] = data;
}

void Scene::set_data_buffer(int index, const Vec3f& data)
{
	data_buffer_[index] = data;
}

void Scene::set_z_buffer(int index, const float val)
{
	z_buffer_[index] = val;
}

void Scene::set_z_buffer(int x, int y, const float val)
{
	z_buffer_[get_index(x, y)] = val;
}

float Scene::get_z_buffer(int index) const 
{
	return z_buffer_[index];
}

float Scene::get_z_buffer(int x, int y) const 
{
	return z_buffer_[get_index(x, y)];
}

void Scene::write() const
{
	FILE* fp = fopen("./output/binary.ppm", "wb");
	(void)fprintf(fp, "P6\n%d %d\n255\n", width_, height_);
	for (auto i = 0; i < height_ * width_; ++i) {
		static unsigned char color[3];
		color[0] = (unsigned char)(255 * std::pow(clamp(0, 1, data_buffer_[i].x), 0.6f));
		color[1] = (unsigned char)(255 * std::pow(clamp(0, 1, data_buffer_[i].y), 0.6f));
		color[2] = (unsigned char)(255 * std::pow(clamp(0, 1, data_buffer_[i].z), 0.6f));
		fwrite(color, 1, 3, fp);
	}
	fclose(fp);

	// Python脚本的路径，如果不在PATH中，需要提供完整路径  
	const char* pythonScriptPath = "python/ppm2jpg.py";
	// PPM图片的路径  
	const char* ppmImagePath = "./output/binary.ppm";
	// 输出的JPG图片路径  
	const char* jpgOutputPath = "./output/output.jpg";

	std::string command = "python " + std::string(pythonScriptPath) + " " + std::string(ppmImagePath) + " " +
		std::string(jpgOutputPath);
	// 执行命令行指令  
	int result = system(command.c_str());

	// 检查执行结果  
	if (result == 0) {
		std::cout << "Python script executed successfully." << std::endl;
	}
	else {
		std::cerr << "Failed to execute Python script." << std::endl;
	}
}

void Scene::useMsaa()
{
	use_msaa_ = true;
	z_buffer_.clear();
	z_buffer_msaa_.resize(width_ * height_);
	for (auto& item : z_buffer_msaa_)
	{
		item.resize(msaa_spp);
	}
}




