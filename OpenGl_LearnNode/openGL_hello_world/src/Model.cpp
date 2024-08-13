#include "Model.h"

#include <Importer.hpp>
#include <postprocess.h>
#include <scene.h>

void Model::Draw(const MyShader& shader)
{
	for(auto& m:meshes_)
	{
		m.draw(shader);
	}
}

void Model::loadModel(const std::string& path)
{
	Assimp::Importer import;
	const aiScene* scene=import.ReadFile(path,aiProcess_Triangulate|aiProcess_FlipUVs);

	if (!scene || scene->mFlags & AI_SCENE_FLAGS_INCOMPLETE || !scene->mRootNode)
	{
		std::cout << "ERROR::ASSIMP::" << import.GetErrorString() << std::endl;
		return;
	}
	directory= path.substr(0, path.find_last_of('/'));

	processNode(scene->mRootNode, scene);
}

void Model::processNode(aiNode* node, const aiScene* scene)
{
	for(unsigned i=0;i<node->mNumMeshes;++i)
	{
		aiMesh* mesh = scene->mMeshes[node->mMeshes[i]];
		meshes_.emplace_back(processMesh(mesh,scene));
	}

	for (unsigned int i = 0; i < node->mNumChildren; ++i)
	{
		processNode(node->mChildren[i], scene);
	}
 }

Mesh Model::processMesh(aiMesh* mesh, const aiScene* scene)
{
	std::vector<Vertex>vertices;
	std::vector<unsigned int>indices;
	std::vector<Texture>textures;

	for (unsigned int i = 0; i < mesh->mNumVertices; ++i)
	{
		Vertex vertex;
		glm::vec3 vec;
		///顶点位置
		vec.x = mesh->mVertices[i].x;
		vec.y = mesh->mVertices[i].y;
		vec.z = mesh->mVertices[i].z;
		vertex.position = vec;

		///顶点法向量
		if (mesh->HasNormals())
		{
			vec.x = mesh->mNormals[i].x;
			vec.y = mesh->mNormals[i].y;
			vec.z = mesh->mNormals[i].z;
			vertex.normal = vec;
		}

		///顶点贴图索引
		if (mesh->mTextureCoords[0])
		{
			glm::vec2 vec2;
			vec2.x = mesh->mTextureCoords[0][i].x;
			vec2.y = mesh->mTextureCoords[0][i].y;
			vertex.texCoords = vec2;
		}
		vertices.emplace_back(vertex);
	}

	for(unsigned int i=0;i<mesh->mNumFaces;++i)
	{
		const aiFace& face = mesh->mFaces[i];
		for (unsigned int j = 0; j < face.mNumIndices; ++j)
		{
			indices.emplace_back(face.mIndices[j]);
		}
	}

	if (mesh->mMaterialIndex > 0)
	{
		aiMaterial* material = scene->mMaterials[mesh->mMaterialIndex];

		std::vector<Texture>diffuseMaps = loadMaterialTextures(material, aiTextureType_DIFFUSE, "texture_diffuse");
		textures.insert(textures.end(), diffuseMaps.begin(), diffuseMaps.end());

		std::vector<Texture>specularMaps = loadMaterialTextures(material, aiTextureType_SPECULAR, "texture_specular");
		textures.insert(textures.end(), specularMaps.begin(), specularMaps.end());
	}
	return Mesh(vertices, indices, textures);
}

std::vector<Texture> Model::loadMaterialTextures(aiMaterial* mat, aiTextureType type, std::string typeName)
{
	std::vector<Texture>textures;

	for(unsigned int i=0;i<mat->GetTextureCount(type);++i)
	{
		aiString str;
		mat->GetTexture(type, i, &str);

		bool skip = false;
		for(int i=0;i<loadTextures_.size();++i)
		{
			if(std::strcmp(loadTextures_[i].path.C_Str(),str.C_Str())==0)
			{
				textures.push_back(loadTextures_[i]);
				skip = true;
				break;
			}
		}
		if(skip)
			continue;
		Texture texture;
		texture.id = TexturefromFile(str.C_Str(), directory.c_str());
		texture.type = typeName;
		texture.path = str;
		textures.emplace_back(texture);
	}
	return textures;
}

unsigned int Model::TexturefromFile(const char* path, const char* directory)
{
	const std::string p = path;
	const std::string d = directory;
	const std::string texturePath = d + "/" + p;
	unsigned int textureId;
	MyShader::loadTexture(texturePath, textureId);
	return textureId;
}
