#include "Scene.h"
#include "global.hpp"
#include "Geometry.h"
#include "Rasterizer.h"
#include "Texture.h"
#include "Mesh.h"
#include "Camera.h"
#include "Timer.h"

//#define DEBUG
#define USEPARALLEL

Scene::Scene()
{
	dataBuffer_.resize(width_ * height_);
	zBuffer_.resize(width_ * height_);
	std::fill(zBuffer_.begin(), zBuffer_.end(), std::numeric_limits<float>::infinity());

	rasterizer_ = std::make_unique<Rasterizer>(this);
	camera_ = std::make_shared<Camera>(this);
}


void Scene::addMesh(const std::shared_ptr<Mesh>& mesh)
{
	std::vector<Geometry*>geometries;
	for (int i = 0; i < mesh->vertices_.size(); i += 3)
	{
		Triangle* t = new Triangle();
		for (int j = 0; j < 3; j++)
		{
			t->setVertex(j, Vec3f(mesh->vertices_[i + j].position_.x, mesh->vertices_[i + j].position_.y, mesh->vertices_[i + j].position_.z));
			t->setNormal(j, Vec3f(mesh->vertices_[i + j].normal_.x, mesh->vertices_[i + j].normal_.y, mesh->vertices_[i + j].normal_.z));
			t->setColor(j, Vec3f(mesh->vertices_[i + j].color_.x, mesh->vertices_[i + j].color_.y, mesh->vertices_[i + j].color_.z));
			t->setTexCoord(j, Vec2f(mesh->vertices_[i + j].textureCoordinate_.x, mesh->vertices_[i + j].textureCoordinate_.y));
		}
		t->boxConstruct();
		geometries.push_back(t);
	}
	auto pair = std::make_pair(mesh, geometries);
	meshCords_.insert(pair);
}

void Scene::doUpDate()
{
	for(const auto& it:meshCords_)
	{
		const Mesh* mesh = it.first.get();

		const Matrix4x4f& mvp = getMVPMat(mesh);
		const Matrix4x4f& mv = getMVMat(mesh);
		int index = 0;
		for (const auto& model : it.second)
		{
			Timer timer;
			auto* triangle = dynamic_cast<Triangle*>(model);
			if (!triangle)
				return;

			Vec4Vector vec = getViewPortPos(triangle->getVertexs(), mvp);
			Vec3Vector points = getPointPos(triangle->getVertexs(), mv, false);
			//Vec3Vector normals = triangle->getNormals();
			Vec3Vector normals = getVertexNormal(triangle->getNormals(), mv);
			//光栅化
			Triangle t;
			//Vec3f color = { k / it.second.size()};
			for (int i = 0; i < 3; ++i)
			{
				Vec3f v = { vec[i].x,vec[i].y,vec[i].z };

				t.setVertex(i, v);
				t.setColor(i, triangle->getColor(i));
				t.setTexCoord(i, triangle->getTexCoord(i));
				t.setNormal(i, normals[i]);
			}
			/*t.setColor(0, Color::Gray);
			t.setColor(1, Color::Gray);
			t.setColor(2, Color::Gray);*/
			t.boxConstruct();
			rasterizeTriangle(&t, points, mesh);
#ifdef DEBUG
			float ms = timer.stop();
			std::cout << "triangle: " << index << " ==> " << static_cast<float>(ms) << " \n";
			std::cout.flush();
			index++;
#endif // DEBUG			
		}
	}
}

void Scene::rasterizeTriangle(Geometry* geometry, const Vec3Vector& points, const Mesh* mesh)
{
	Triangle* triangle = dynamic_cast<Triangle*>(geometry);
	if(!triangle)
		return;

	auto v = triangle->getVertexs();
	int edgeBorder = 0;
	int left = std::min(v[0].x, std::min(v[1].x, v[2].x)) - edgeBorder;
	int Right = std::max(v[0].x, std::max(v[1].x, v[2].x)) + edgeBorder;
	int top = std::max(v[0].y, std::max(v[1].y, v[2].y)) + edgeBorder;
	int bottom = std::min(v[0].y, std::min(v[1].y, v[2].y)) - edgeBorder;
#ifdef USEPARALLEL
	#pragma omp parallel for
#endif // !DEBUG
	for(int i=0;i< width_;i++)
	{
		for(int j=0;j<height_;j++)
		{
			if (i<left || i>Right || j<bottom || j>top)
			{
				continue;
			}
			rasterizer_->rasterizeTriangle(i, j, triangle, points, mesh);
		}
	}
}

void Scene::setFragmentShaderFunction(const std::function<Vec3f(fragment_shader_payload)>& fun) const noexcept
{
	if (rasterizer_)
		rasterizer_->setFragmentShaderFunction(fun);
}

void Scene::setVertexShaderFunction(const std::function<Vec3f(vertex_shader_payload)>& fun) const noexcept
{
	if (rasterizer_)
		rasterizer_->setVertexShaderFunction(fun);
}



Vec4Vector Scene::getViewPortPos(const Vec3Vector& trianglePos, const Matrix4x4f& mat) const
{
	Vec4Vector res;
	for (auto& p : trianglePos)
	{
		Vec4f v = mat * Vec4f(p, 1.0);
		res.push_back(v);
	}
	viewportMatrix(res);
	return res;
}

Vec3Vector Scene::getPointPos(const Vec3Vector& trianglePos, const Matrix4x4f& mat, bool isVec) const
{
	Vec4Vector temp;
	float w = isVec ? 0.0 : 1.0;
	for (auto& p : trianglePos)
	{
		Vec4f v = mat * Vec4f(p, w);
		temp.push_back(v);
	}
	Vec3Vector res(temp.size(), 0);
	std::transform(temp.begin(), temp.end(), res.begin(), [](const Vec4f& v)
		{
			return v.head();
		});
	return res;
}

Vec3Vector Scene::getVertexNormal(const Vec3Vector& vertexNormal, const Matrix4x4f& mat) const
{
	Vec3Vector normals = getPointPos(vertexNormal, mat, true);
	for (auto& n : normals)
	{
		n.normalize();
	}
	/*Matrix4x4f mv = camera_->getViewMat() * mesh->modelMatrix_;

	Vec3Vectortemp;
	std::vector<Eigen::Vector3f>res;
	for (auto& p : trianglePos)
	{
		Eigen::Vector4f vec4f = mv * p.to_vec4(1.0);
		temp.push_back(vec4f);
	}
	std::transform(temp.begin(), temp.end(), res.begin(), [](auto& v)
		{
			return v.template head<3>();
		});
	return res;*/
	return normals;
}

void  Scene::viewportMatrix(Vec4Vector& vec) const
{
	float f1 = (far_ + near_) / 2.0f;
	float f2 = (far_ - near_) / 2.0f;

	for(auto& v: vec)
	{
		v /= v.W();
	}

	for (auto& v : vec)
	{
		v.X() = 0.5f * width_ * (v.X() + 1.0f);
		v.Y() = 0.5f * height_ * (v.Y() + 1.0f);
		v.Z() = v.Z() * f2 + f1;
	}
}

Matrix4x4f Scene::getMVPMat(const Mesh* mesh) const
{
	return camera_->getProjectionMat() * camera_->getViewMat() * mesh->modelMatrix_;
}

Matrix4x4f Scene::getMVMat(const Mesh* mesh) const
{
	return camera_->getViewMat() * mesh->modelMatrix_;
}

Matrix4x4f Scene::setViewMatrix(Vec3f eyePos)
{
	Matrix4x4f view = Matrix4x4f::Identity();

	Matrix4x4f translate;
	translate = Matrix4x4f{
		1, 0, 0, -eyePos[0],
		0, 1, 0, -eyePos[1],
		0, 0, 1, -eyePos[2],
		0, 0, 0, 1 };

	view = translate * view;

	return view;
}

void Scene::clear()
{
	std::fill(dataBuffer_.begin(), dataBuffer_.end(), Vec3f{ 0, 0, 0 });
	std::fill(zBuffer_.begin(), zBuffer_.end(), std::numeric_limits<float>::infinity());
	if(useMsaa_)
	{
		for (auto& item : zBufferMsaa_)
		{
			item.resize(msaaSpp_);
			std::fill(item.begin(), item.end(), std::numeric_limits<float>::infinity());
		}
	}
	
}


int Scene::getIndex(int x, int y) const
{
	return (height_ - y - 1) * width_ + x;
}

int Scene::getMsaaIndex(int h, int v) const
{
	return v + h * msaaV_;
}

Vec3f Scene::getDataBuffer(int index) const
{
	return dataBuffer_[index];
}

Vec3f Scene::getDataBuffer(int x, int y) const
{
	return dataBuffer_[getIndex(x, y)];
}

void Scene::setDataBuffer(int x, int y, const Vec3f& data)
{
	dataBuffer_[getIndex(x, y)] = data;
}

void Scene::setDataBuffer(int index, const Vec3f& data)
{
	dataBuffer_[index] = data;
}

void Scene::setZBuffer(int index, const float val)
{
	zBuffer_[index] = val;
}

void Scene::setZBuffer(int x, int y, const float val)
{
	zBuffer_[getIndex(x, y)] = val;
}

float Scene::getZBuffer(int index) const 
{
	return zBuffer_[index];
}

float Scene::getZBuffer(int x, int y) const 
{
	return zBuffer_[getIndex(x, y)];
}

void Scene::setMassZBuffer(int x, int y, int h, int v, const float val)
{
	zBufferMsaa_[getIndex(x, y)][getMsaaIndex(h, v)] = val;
}

void Scene::setMassZBuffer(int index, int h, int v, const float val)
{
	zBufferMsaa_[index][getMsaaIndex(h, v)] = val;
}

float Scene::getMassZBuffer(int x, int y, int h, int v) const
{
	return zBufferMsaa_[getIndex(x, y)][getMsaaIndex(h, v)];
}

float Scene::getMassZBuffer(int index, int h, int v) const
{
	return zBufferMsaa_[index][getMsaaIndex(h, v)];
}

void Scene::write() const
{
	FILE* fp;
	fopen_s(&fp, "./output/binary.ppm", "wb");
	(void)fprintf(fp, "P6\n%d %d\n255\n", width_, height_);
	for (auto i = 0; i < height_ * width_; ++i) {
		static unsigned char color[3];
		color[0] = (unsigned char)(255 * std::pow(clamp(0, 1, dataBuffer_[i].x), 0.6f));
		color[1] = (unsigned char)(255 * std::pow(clamp(0, 1, dataBuffer_[i].y), 0.6f));
		color[2] = (unsigned char)(255 * std::pow(clamp(0, 1, dataBuffer_[i].z), 0.6f));
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

void Scene::useMsaa(int sizeH, int sizeV)
{
	useMsaa_ = true;
	msaaH_ = sizeH;
	msaaV_ = sizeV;
	msaaSpp_ = sizeH * sizeV;
	//zBuffer_.clear();
	zBufferMsaa_.resize(width_ * height_);
	for (auto& item : zBufferMsaa_)
	{
		item.resize(msaaSpp_);
		std::fill(item.begin(), item.end(), std::numeric_limits<float>::infinity());
	}
	
}




