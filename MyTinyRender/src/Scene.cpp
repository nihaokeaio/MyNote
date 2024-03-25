#include "Scene.h"
#include "global.hpp"
#include "Geometry.h"

#include <Eigen/Eigen>

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
		Triangle* triangle = dynamic_cast<Triangle*>(model);
		if (!triangle)
			return;
		//投影变换

		//视口变化
		Eigen::Matrix4f modelMatrix = Eigen::Matrix4f::Identity();
		Eigen::Matrix4f viewMatrix = setViewMatrix(Vec3f(0, 0, 5.f));
		Eigen::Matrix4f pojMatrix = projectMatrix();


		Eigen::Matrix4f mvp = pojMatrix * viewMatrix * modelMatrix;

		std::vector<Eigen::Vector4f> vec = triangle->setModelMatrix(mvp);
		viewportMatrix(vec);

		//光栅化
		Triangle t;
		for(int i=0;i<3;++i)
		{
			Vec3f v = { vec[i].x(),vec[i].y(),vec[i].z() };
			t.setVertex(i, v);
		}
		t.boxConstruct();
		t.setColors(triangle->getColors());
		rasterizeTriangle(&t);
	}
}

void Scene::rasterizeTriangle(Geometry* geometry)
{
	Triangle* triangle = dynamic_cast<Triangle*>(geometry);
	if(!triangle)
		return;

	for(int i=0;i< width_;i++)
	{
		for(int j=0;j<height_;j++)
		{
			if(i==50 && j==80)
			{
				int x = 1;
			}
			const Vec2f pixel{ i + 0.5f,j + 0.5f };
			if(!triangle->inBoundBox(pixel))
				continue;
			if(insideTriangle(pixel.x,pixel.y,triangle->vertexs()))
			{
				RasterizerWay way = RasterizerWay::BARYCENTRIC;

				switch (way)
				{
				case RasterizerWay::DEFAULT:
					{
						///使用深度图
						///
						if (triangle->vertexs()->Z() < get_z_buffer(i, j))
						{
							set_data_buffer(i, j, triangle->getColor());
							set_z_buffer(i, j, triangle->vertexs()->Z());
						}
						break;
					}
				case RasterizerWay::BARYCENTRIC:
					{
						///使用重心坐标
						auto [alpha, beta, gamma] = computeBarycentric2D(pixel.x, pixel.y, triangle->vertexs());
						const Vec3f& color = alpha * triangle->getColor(0) + beta * triangle->getColor(1) + gamma * triangle->getColor(2);
						const float& pixelZ = alpha * triangle->vertexs(0).z + beta * triangle->vertexs(1).z + gamma * triangle->vertexs(2).z;
						if (pixelZ <= get_z_buffer(i, j))
						{
							set_data_buffer(i, j, color);
							set_z_buffer(i, j, pixelZ);
						}
						break;
					}
				case RasterizerWay::MASS:
					break;
				}
			}
		}
	}
}

Eigen::Matrix4f Scene::projectMatrix() const
{
	Eigen::Matrix4f projection = Eigen::Matrix4f::Identity();
	float zNear = near;
	float zFar = far;
	projection << zNear, 0, 0, 0,
		0, zNear, 0, 0,
		0, 0, zNear + zFar, -zNear * zFar,
		0, 0, 1, 0;
	Eigen::Matrix4f scale = Eigen::Matrix4f::Identity();
	float H = 2 * -zNear * tan(eye_fov / 180 * M_PI / 2);
	float W =aspect_ratio * H;

	Eigen::Matrix4f ortho = Eigen::Matrix4f::Identity();

	ortho << 2 / W, 0, 0, 0,
		0, 2 / H, 0, 0,
		0, 0, 2 / (zNear - zFar), 0,
		0, 0, 0, 1;

	projection = scale * ortho * projection;

	return projection;
}

void  Scene::viewportMatrix(std::vector<Eigen::Vector4f>& vec)
{
	float f1 = (far + near) / 2.0f;
	float f2 = (far - near) / 2.0f;

	for(auto& v: vec)
	{
		v /= v.w();
	}

	for (auto& v : vec)
	{
		v.x() = 0.5f * width_ * (v.x() + 1.0f);
		v.y() = 0.5f * height_ * (v.y() + 1.0f);
		v.z() = v.z() * f2 + f1;
	}
}

Eigen::Matrix4f Scene::setViewMatrix(Vec3f eyePos)
{
	Eigen::Matrix4f view = Eigen::Matrix4f::Identity();

	Eigen::Matrix4f translate;
	translate << 1, 0, 0, -eyePos[0],
		0, 1, 0, -eyePos[1],
		0, 0, 1, -eyePos[2],
		0, 0, 0, 1;

	view = translate * view;

	return view;
}

int Scene::get_index(int x, int y) const
{
	return (height_ - y - 1) * width_ + x;
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




