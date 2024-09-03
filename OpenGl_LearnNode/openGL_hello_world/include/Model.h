#pragma once
#include <material.h>
#include <vector>

#include "Mesh.h"


struct aiMaterial;
struct aiNode;
struct aiMesh;
struct aiScene;

class Model
{
public:
    /*  ����   */
    Model(const char* path)
    {
        loadModel(path);
    }
    void Draw(const MyShader& shader);
    void DrawInts(const MyShader& shader, int intsCount);

    void attachAttribPointer(const std::vector<glm::mat4>& nums);
private:
    /// ģ������  
    std::vector<Mesh> meshes_;
    std::vector<Texture>loadTextures_;

    std::string directory;
    ///����
    void loadModel(const std::string& path);
    void processNode(aiNode* node, const aiScene* scene);
    Mesh processMesh(aiMesh* mesh, const aiScene* scene);
    std::vector<Texture> loadMaterialTextures(aiMaterial* mat, aiTextureType type, std::string typeName);
    unsigned int TexturefromFile(const char* path, const char* directory);
};


