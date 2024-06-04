#pragma once

#include "global.hpp"
#include "Mesh.h"
#include <vector>
#include <string>
#include <fstream>

class LoaderMesh
{
public:
    LoaderMesh() = default;

    LoaderMesh(const std::string& filePath)
    {
        loadObjFile(filePath);
    }

    bool loadObjFile(const std::string& filePath)
    {
        if (filePath.substr(filePath.size() - 4, 4) != ".obj")
            return false;

        std::ifstream file(filePath);

        if (!file.is_open())
            return false;

        LoadedMeshes_.clear();
        LoadedVertices_.clear();
        LoadedIndices_.clear();

        std::vector<Vec3f> Positions;
        std::vector<Vec2f> TCoords;
        std::vector<Vec3f> Normals;

        std::vector<Vertex> Vertices;
        std::vector<unsigned int> Indices;

        std::vector<std::string> MeshMatNames;

        std::string meshname;
        Mesh tempMesh;

#ifdef OBJL_CONSOLE_OUTPUT
        const unsigned int outputEveryNth = 1000;
        unsigned int outputIndicator = outputEveryNth;
#endif

        std::string curline;
        while (std::getline(file, curline))
        {
            if (firstToken(curline) == "o" || firstToken(curline) == "g" || curline[0] == 'g')
            {
                if (firstToken(curline) == "o" || firstToken(curline) == "g")
                {
                    meshname = tail(curline);
                }
                else
                {
                    meshname = "unnamed";
                }
            }

            // Generate a Vertex position_
            if (firstToken(curline) == "v")
            {
                std::vector<std::string> spos;
                Vec3f vpos;
                split(tail(curline), spos, " ");

                vpos.x = std::stof(spos[0]);
                vpos.y = std::stof(spos[1]);
                vpos.z = std::stof(spos[2]);

                Positions.push_back(vpos);
            }

            // Generate a Vertex Texture Coordinate
            if (firstToken(curline) == "vt")
            {
                std::vector<std::string> stex;
                Vec2f vtex;
                split(tail(curline), stex, " ");

                vtex.x = std::stof(stex[0]);
                vtex.y = std::stof(stex[1]);

                TCoords.push_back(vtex);
            }


            // Generate a Vertex normal_;
            if (firstToken(curline) == "vn")
            {
                std::vector<std::string> snor;
                Vec3f vnor;
                split(tail(curline), snor, " ");

                vnor.x = std::stof(snor[0]);
                vnor.y= std::stof(snor[1]);
                vnor.z = std::stof(snor[2]);

                Normals.push_back(vnor);
            }
            // Generate a Face (vertices & indices)
            if (firstToken(curline) == "f")
            {
                // Generate the vertices
                std::vector<Vertex> vVerts;
                GenVerticesFromRawOBJ(vVerts, Positions, TCoords, Normals, curline);

                // Add Vertices
                for (int i = 0; i < int(vVerts.size()); i++)
                {
                    Vertices.push_back(vVerts[i]);

                    LoadedVertices_.push_back(vVerts[i]);
                }

                std::vector<unsigned int> iIndices;

                VertexTriangluation(iIndices, vVerts);

                // Add Indices
                for (int i = 0; i < int(iIndices.size()); i++)
                {
                    unsigned int indnum = (unsigned int)((Vertices.size()) - vVerts.size()) + iIndices[i];
                    Indices.push_back(indnum);

                    indnum = (unsigned int)((LoadedVertices_.size()) - vVerts.size()) + iIndices[i];
                    LoadedIndices_.push_back(indnum);

                }
            }
            // Get Mesh Material Name
            if (firstToken(curline) == "usemtl")
            {
                MeshMatNames.push_back(tail(curline));

                // Create new Mesh, if Material changes within a group
                if (!Indices.empty() && !Vertices.empty())
                {
                    // Create Mesh
                    tempMesh = Mesh(Vertices, Indices);
                    tempMesh.name_ = meshname;
                    int i = 2;
                    while (1) {
                        tempMesh.name_ = meshname + "_" + std::to_string(i);

                        for (auto& m : LoadedMeshes_)
                            if (m.name_ == tempMesh.name_)
                                continue;
                        break;
                    }

                    // Insert Mesh
                    LoadedMeshes_.push_back(tempMesh);

                    // Cleanup
                    Vertices.clear();
                    Indices.clear();
                }

#ifdef OBJL_CONSOLE_OUTPUT
                outputIndicator = 0;
#endif
            }
            // Load Materials
            if (firstToken(curline) == "mtllib")
            {
                // Generate LoadedMaterial

                // Generate a path to the material file
                std::vector<std::string> temp;
                split(filePath, temp, "/");

                std::string pathtomat = "";

                if (temp.size() != 1)
                {
                    for (int i = 0; i < temp.size() - 1; i++)
                    {
                        pathtomat += temp[i] + "/";
                    }
                }


                pathtomat += tail(curline);


                // Load Materials
                LoadMaterials(pathtomat);
            }
        }


        // Deal with last mesh

        if (!Indices.empty() && !Vertices.empty())
        {
            // Create Mesh
            tempMesh = Mesh(Vertices, Indices);
            tempMesh.name_ = meshname;

            // Insert Mesh
            LoadedMeshes_.push_back(tempMesh);
        }

        file.close();

        // Set Materials for each Mesh
        for (int i = 0; i < MeshMatNames.size(); i++)
        {
            std::string matname = MeshMatNames[i];

            // Find corresponding material name in loaded materials
            // when found copy material variables into mesh material
            for (int j = 0; j < LoadedMaterials_.size(); j++)
            {
                if (LoadedMaterials_[j].name_ == matname)
                {
                    LoadedMeshes_[i].material = LoadedMaterials_[j];
                    break;
                }
            }
        }

        if (LoadedMeshes_.empty() && LoadedVertices_.empty() && LoadedIndices_.empty())
        {
            return false;
        }
        else
        {
            return true;
        }
    }


private:
    // Generate vertices from a list of positions,
	//	tcoords, normals and a face line
    void GenVerticesFromRawOBJ(std::vector<Vertex>& oVerts,
        const std::vector<Vec3f>& iPositions,
        const std::vector<Vec2f>& iTCoords,
        const std::vector<Vec3f>& iNormals,
        const std::string& icurline)
    {
        std::vector<std::string> sface, svert;
        Vertex vVert;
        split(tail(icurline), sface, " ");

        bool noNormal = false;

        // For every given vertex do this
        for (int i = 0; i < int(sface.size()); i++)
        {
            // See What type the vertex is.
            int vtype;

            split(sface[i], svert, "/");

            // Check for just position - v1
            if (svert.size() == 1)
            {
                // Only position
                vtype = 1;
            }

            // Check for position & texture - v1/vt1
            if (svert.size() == 2)
            {
                // position_ & Texture
                vtype = 2;
            }

            // Check for position_, Texture and normal_ - v1/vt1/vn1
            // or if position_ and normal_ - v1//vn1
            if (svert.size() == 3)
            {
                if (svert[1] != "")
                {
                    // position_, Texture, and normal_
                    vtype = 4;
                }
                else
                {
                    // position_ & normal_
                    vtype = 3;
                }
            }

            // Calculate and store the vertex
            switch (vtype)
            {
            case 1: // P
            {
                vVert.position_ = getElement(iPositions, svert[0]);
                vVert.textureCoordinate_ = Vec2f(0, 0);
                noNormal = true;
                oVerts.push_back(vVert);
                break;
            }
            case 2: // P/T
            {
                vVert.position_ = getElement(iPositions, svert[0]);
                vVert.textureCoordinate_ = getElement(iTCoords, svert[1]);
                noNormal = true;
                oVerts.push_back(vVert);
                break;
            }
            case 3: // P//N
            {
                vVert.position_ = getElement(iPositions, svert[0]);
                vVert.textureCoordinate_ = Vec2f(0, 0);
                vVert.normal_ = getElement(iNormals, svert[2]);
                oVerts.push_back(vVert);
                break;
            }
            case 4: // P/T/N
            {
                vVert.position_ = getElement(iPositions, svert[0]);
                vVert.textureCoordinate_ = getElement(iTCoords, svert[1]);
                vVert.normal_ = getElement(iNormals, svert[2]);
                oVerts.push_back(vVert);
                break;
            }
            default:
            {
                break;
            }
            }
        }

        // take care of missing normals
        // these may not be truly acurate but it is the
        // best they get for not compiling a mesh with normals
        if (noNormal)
        {
	        const Vec3f A = oVerts[0].position_ - oVerts[1].position_;
	        const Vec3f B = oVerts[2].position_ - oVerts[1].position_;

	        const Vec3f normal = A.cross(B);

            for (int i = 0; i < int(oVerts.size()); i++)
            {
                oVerts[i].normal_ = normal;
            }
        }
    }

    // Triangulate a list of vertices into a face by printing
    //	inducies corresponding with triangles within it
    void VertexTriangluation(std::vector<unsigned int>& oIndices,
        const std::vector<Vertex>& iVerts)
    {
        // If there are 2 or less verts,
        // no triangle can be created,
        // so exit
        if (iVerts.size() < 3)
        {
            return;
        }
        // If it is a triangle no need to calculate it
        if (iVerts.size() == 3)
        {
            oIndices.push_back(0);
            oIndices.push_back(1);
            oIndices.push_back(2);
            return;
        }

        // Create a list of vertices
        std::vector<Vertex> tVerts = iVerts;

        while (true)
        {
            // For every vertex
            for (int i = 0; i < int(tVerts.size()); i++)
            {
                // pPrev = the previous vertex in the list
                Vertex pPrev;
                if (i == 0)
                {
                    pPrev = tVerts[tVerts.size() - 1];
                }
                else
                {
                    pPrev = tVerts[i - 1];
                }

                // pCur = the current vertex;
                Vertex pCur = tVerts[i];

                // pNext = the next vertex in the list
                Vertex pNext;
                if (i == tVerts.size() - 1)
                {
                    pNext = tVerts[0];
                }
                else
                {
                    pNext = tVerts[i + 1];
                }

                // Check to see if there are only 3 verts left
                // if so this is the last triangle
                if (tVerts.size() == 3)
                {
                    // Create a triangle from pCur, pPrev, pNext
                    for (int j = 0; j < int(tVerts.size()); j++)
                    {
                        if (iVerts[j].position_ == pCur.position_)
                            oIndices.push_back(j);
                        if (iVerts[j].position_ == pPrev.position_)
                            oIndices.push_back(j);
                        if (iVerts[j].position_ == pNext.position_)
                            oIndices.push_back(j);
                    }

                    tVerts.clear();
                    break;
                }
                if (tVerts.size() == 4)
                {
                    // Create a triangle from pCur, pPrev, pNext
                    for (int j = 0; j < int(iVerts.size()); j++)
                    {
                        if (iVerts[j].position_ == pCur.position_)
                            oIndices.push_back(j);
                        if (iVerts[j].position_ == pPrev.position_)
                            oIndices.push_back(j);
                        if (iVerts[j].position_ == pNext.position_)
                            oIndices.push_back(j);
                    }

                    Vec3f tempVec;
                    for (int j = 0; j < int(tVerts.size()); j++)
                    {
                        if (tVerts[j].position_ != pCur.position_
                            && tVerts[j].position_ != pPrev.position_
                            && tVerts[j].position_ != pNext.position_)
                        {
                            tempVec = tVerts[j].position_;
                            break;
                        }
                    }

                    // Create a triangle from pCur, pPrev, pNext
                    for (int j = 0; j < int(iVerts.size()); j++)
                    {
                        if (iVerts[j].position_ == pPrev.position_)
                            oIndices.push_back(j);
                        if (iVerts[j].position_ == pNext.position_)
                            oIndices.push_back(j);
                        if (iVerts[j].position_ == tempVec)
                            oIndices.push_back(j);
                    }

                    tVerts.clear();
                    break;
                }

                // If Vertex is not an interior vertex
                const float angle =(pPrev.position_ - pCur.position_).angleBetweenVec(pNext.position_ - pCur.position_) * (180 / 3.14159265359);
                if (angle <= 0 && angle >= 180)
                    continue;

                // If any vertices are within this triangle
                bool inTri = false;
                for (int j = 0; j < int(iVerts.size()); j++)
                {
                    if (inTriangle(iVerts[j].position_, pPrev.position_, pCur.position_, pNext.position_)
                        && iVerts[j].position_ != pPrev.position_
                        && iVerts[j].position_ != pCur.position_
                        && iVerts[j].position_ != pNext.position_)
                    {
                        inTri = true;
                        break;
                    }
                }
                if (inTri)
                    continue;

                // Create a triangle from pCur, pPrev, pNext
                for (int j = 0; j < int(iVerts.size()); j++)
                {
                    if (iVerts[j].position_ == pCur.position_)
                        oIndices.push_back(j);
                    if (iVerts[j].position_ == pPrev.position_)
                        oIndices.push_back(j);
                    if (iVerts[j].position_ == pNext.position_)
                        oIndices.push_back(j);
                }

                // Delete pCur from the list
                for (int j = 0; j < int(tVerts.size()); j++)
                {
                    if (tVerts[j].position_ == pCur.position_)
                    {
                        tVerts.erase(tVerts.begin() + j);
                        break;
                    }
                }

                // reset i to the start
                // -1 since loop will add 1 to it
                i = -1;
            }

            // if no triangles were created
            if (oIndices.empty())
                break;

            // if no more vertices
            if (tVerts.empty())
                break;
        }
    }

    // Load Materials from .mtl file
    bool LoadMaterials(const std::string& path)
    {
        // If the file is not a material file return false
        if (path.substr(path.size() - 4, path.size()) != ".mtl")
            return false;

        std::ifstream file(path);

        // If the file is not found return false
        if (!file.is_open())
            return false;

        Material tempMaterial;

        bool listening = false;

        // Go through each line looking for material variables
        std::string curline;
        while (std::getline(file, curline))
        {
            // new material and material name
            if (firstToken(curline) == "newmtl")
            {
                if (!listening)
                {
                    listening = true;

                    if (curline.size() > 7)
                    {
                        tempMaterial.name_ = tail(curline);
                    }
                    else
                    {
                        tempMaterial.name_ = "none";
                    }
                }
                else
                {
                    // Generate the material

                    // Push Back loaded Material
                    LoadedMaterials_.push_back(tempMaterial);

                    // Clear Loaded Material
                    tempMaterial = Material();

                    if (curline.size() > 7)
                    {
                        tempMaterial.name_ = tail(curline);
                    }
                    else
                    {
                        tempMaterial.name_ = "none";
                    }
                }
            }
            // Ambient Color
            if (firstToken(curline) == "Ka")
            {
                std::vector<std::string> temp;
                split(tail(curline), temp, " ");

                if (temp.size() != 3)
                    continue;

                tempMaterial.Ka.x = std::stof(temp[0]);
                tempMaterial.Ka.y = std::stof(temp[1]);
                tempMaterial.Ka.z = std::stof(temp[2]);
            }
            // Diffuse Color
            if (firstToken(curline) == "Kd")
            {
                std::vector<std::string> temp;
                split(tail(curline), temp, " ");

                if (temp.size() != 3)
                    continue;

                tempMaterial.Kd.x = std::stof(temp[0]);
                tempMaterial.Kd.y = std::stof(temp[1]);
                tempMaterial.Kd.z = std::stof(temp[2]);
            }
            // Specular Color
            if (firstToken(curline) == "Ks")
            {
                std::vector<std::string> temp;
                split(tail(curline), temp, " ");

                if (temp.size() != 3)
                    continue;

                tempMaterial.Ks.x = std::stof(temp[0]);
                tempMaterial.Ks.y = std::stof(temp[1]);
                tempMaterial.Ks.z = std::stof(temp[2]);
            }
            // Specular Exponent
            if (firstToken(curline) == "Ns")
            {
                tempMaterial.Ns = std::stof(tail(curline));
            }
            // Optical Density
            if (firstToken(curline) == "Ni")
            {
                tempMaterial.Ni = std::stof(tail(curline));
            }
            // Dissolve
            if (firstToken(curline) == "d")
            {
                tempMaterial.d = std::stof(tail(curline));
            }
            // Illumination
            if (firstToken(curline) == "illum")
            {
                tempMaterial.illum = std::stoi(tail(curline));
            }
            // Ambient Texture Map
            if (firstToken(curline) == "map_Ka")
            {
                tempMaterial.map_Ka = tail(curline);
            }
            // Diffuse Texture Map
            if (firstToken(curline) == "map_Kd")
            {
                tempMaterial.map_Kd = tail(curline);
            }
            // Specular Texture Map
            if (firstToken(curline) == "map_Ks")
            {
                tempMaterial.map_Ks = tail(curline);
            }
            // Specular Hightlight Map
            if (firstToken(curline) == "map_Ns")
            {
                tempMaterial.map_Ns = tail(curline);
            }
            // Alpha Texture Map
            if (firstToken(curline) == "map_d")
            {
                tempMaterial.map_d = tail(curline);
            }
            // Bump Map
            if (firstToken(curline) == "map_Bump" || firstToken(curline) == "map_bump" || firstToken(curline) == "bump")
            {
                tempMaterial.map_bump = tail(curline);
            }
        }

        // Deal with last material

        // Push Back loaded Material
        LoadedMaterials_.push_back(tempMaterial);

        // Test to see if anything was loaded
        // If not return false
        if (LoadedMaterials_.empty())
            return false;
        // If so return true
        else
            return true;
    }
public:
    // Loaded Mesh Objects
    std::vector<Mesh> LoadedMeshes_;
    // Loaded Vertex Objects
    std::vector<Vertex> LoadedVertices_;
    // Loaded Index Positions
    std::vector<unsigned int> LoadedIndices_;
    // Loaded Material Objects
    std::vector<Material> LoadedMaterials_;
};
