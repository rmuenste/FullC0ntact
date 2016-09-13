/***************************************************************************
*   Copyright (C) 2006 by Raphael Münster   *
*   raphael@Cortez   *
*                                                                         *
*   This program is free software; you can redistribute it and/or modify  *
*   it under the terms of the GNU General Public License as published by  *
*   the Free Software Foundation; either version 2 of the License, or     *
*   (at your option) any later version.                                   *
*                                                                         *
*   This program is distributed in the hope that it will be useful,       *
*   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
*   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
*   GNU General Public License for more details.                          *
*                                                                         *
*   You should have received a copy of the GNU General Public License     *
*   along with this program; if not, write to the                         *
*   Free Software Foundation, Inc.,                                       *
*   59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.             *
***************************************************************************/

//===================================================
//					INCLUDES
//===================================================


#include "3dmesh.h"


namespace i3d {

  Mesh3D::Mesh3D(void)
  {
    valid_ = false;
    indices_ = nullptr;

    com_ = VECTOR3(0, 0, 0);
    transform_.SetIdentity();

    triangleAABBs_ = nullptr;
    indices_ = nullptr;

    texIndices_ = nullptr;

  }//end constructor

  Mesh3D::Mesh3D(char *strName)
  {
    int i = 0;
    do{
      fileName_[i] = strName[i];
    } while (strName[i++] != 0);

    indices_ = nullptr;

    com_ = VECTOR3(0, 0, 0);

    transform_.SetIdentity();

    triangleAABBs_ = nullptr;

    texIndices_ = nullptr;

  }//end constructor

  Mesh3D::~Mesh3D(void)
  {

    if (indices_ != nullptr)
      delete[] indices_;

    indices_ = nullptr;

    if (texIndices_ != nullptr)
    {
      delete[] texIndices_;
      texIndices_ = nullptr;
    }


    if (triangleAABBs_ != nullptr)
      delete[] triangleAABBs_;

    triangleAABBs_ = nullptr;

  }//end deconstructor

  Mesh3D::Mesh3D(const Mesh3D &pMesh)
  {
    this->textured_ = pMesh.textured_;
    this->valid_ = pMesh.valid_;
    this->matId_ = pMesh.matId_;
    this->numFaces_ = pMesh.numFaces_;
    this->numTexCoords_ = pMesh.numTexCoords_;
    this->numVerts_ = pMesh.numVerts_;
    this->faces_ = pMesh.faces_;
    this->indices_ = pMesh.indices_;
    this->texCoords_ = pMesh.texCoords_;
    this->vertexNormals_ = pMesh.vertexNormals_;
    this->vertices_ = pMesh.vertices_;
    this->box_ = pMesh.box_;
    this->orderedTexCoords_ = pMesh.orderedTexCoords_;

    fileName_ = pMesh.fileName_;

    if (pMesh.indices_ != nullptr)
    {
      int numIndices = 3 * this->faces_.size();
      indices_ = new unsigned int[numIndices];
      memcpy(indices_, pMesh.indices_, numIndices);
    }
    else
    {
      texIndices_ = nullptr;
    }

    if (pMesh.texIndices_ != nullptr)
    {
      int numIndices = 3 * this->faces_.size();
      texIndices_ = new unsigned int[numIndices];
      memcpy(texIndices_, pMesh.texIndices_, numIndices);
    }
    else
    {
      texIndices_ = nullptr;
    }

    triangleAABBs_ = nullptr;

    com_ = pMesh.com_;
    transform_ = pMesh.transform_;

  }

  void Mesh3D::generateTriangleBoundingBoxes()
  {
    if (triangleAABBs_ == nullptr)
      triangleAABBs_ = new AABB3r[numFaces_];

    for (int i = 0; i < (int)faces_.size(); i++)
    {
      int vi0 = faces_[i][0];
      int vi1 = faces_[i][1];
      int vi2 = faces_[i][2];

      VECTOR3 minVec = vertices_[vi0];
      VECTOR3 maxVec = vertices_[vi0];

      for (int j = 1; j < 2; j++)
      {
        VECTOR3 &v = vertices_[faces_[i][j]];

        if (v.x < minVec.x)
          minVec.x = v.x;

        if (v.y < minVec.y)
          minVec.y = v.y;

        if (v.z < minVec.z)
          minVec.z = v.z;

        if (v.x > maxVec.x)
          maxVec.x = v.x;

        if (v.y > maxVec.y)
          maxVec.y = v.y;

        if (v.z > maxVec.z)
          maxVec.z = v.z;

        triangleAABBs_[i].init(minVec, maxVec);

      }

    }

  }

  void Mesh3D::calcFakeVertexNormals()
  {

    using namespace std;
    std::vector<Vec3> pNormals;
    vertexNormals_.clear();
    vertexNormals_ = std::vector<Vec3>(3 * faces_.size());
    //calculate the face normals in a
    //first loop
    for (int i = 0; i < (int)vertices_.size(); i += 3)
    {
      //get the vertex indices of the face
      VECTOR3 v0 = vertices_[i];
      VECTOR3 v1 = vertices_[i + 1];
      VECTOR3 v2 = vertices_[i + 2];

      //create 2 vectors, the face normal will be
      //perpendicular to those
      VECTOR3 vV1 = VECTOR3::createVector(v0, v2);
      VECTOR3 vV2 = VECTOR3::createVector(v2, v1);

      //Calculate and assign the face normal		
      Vec3 faceNormal = Vec3::Cross(vV1, vV2);
      faceNormal.normalize();
      vertexNormals_[i] = faceNormal;
      vertexNormals_[i + 1] = faceNormal;
      vertexNormals_[i + 2] = faceNormal;
    }//end for

  }

  void Mesh3D::calcVertexNormals()
  {

    using namespace std;
    //correctely size the vectors
    vector<int> *pFacesAtVertex = new vector<int>[vertices_.size()];
    std::vector<Vec3> pNormals;
    pNormals.clear();
    vertexNormals_.clear();
    //calculate the face normals in a
    //first loop
    for (int i = 0; i < (int)faces_.size(); i++)
    {
      //get the vertex indices of the face
      int vi0 = faces_[i][0];
      int vi1 = faces_[i][1];
      int vi2 = faces_[i][2];

      //remember the face index
      pFacesAtVertex[vi0].push_back(i);
      pFacesAtVertex[vi1].push_back(i);
      pFacesAtVertex[vi2].push_back(i);

      VECTOR3 v0 = vertices_[vi0];
      VECTOR3 v1 = vertices_[vi1];
      VECTOR3 v2 = vertices_[vi2];

      //create 2 vectors, the face normal will be
      //perpendicular to those
      VECTOR3 vV1 = VECTOR3::createVector(v0, v2);
      VECTOR3 vV2 = VECTOR3::createVector(v2, v1);

      ////Calculate and assign the face normal		
      pNormals.push_back(VECTOR3::Cross(vV1, vV2));

    }//end for

    //in this 2nd loop calculate the vertex normals
    for (int i = 0; i < (int)vertices_.size(); i++)
    {

      VECTOR3 vSum(0, 0, 0);
      int count = 0;

      for (int j = 0; j < (int)pFacesAtVertex[i].size(); j++)
      {
        vSum += pNormals[pFacesAtVertex[i][j]];
        count++;
      }//end for

      //divide by the number of neighboring face
      //and normalize
      vSum /= Real(count);
      vSum.Normalize();
      vertexNormals_.push_back(-vSum);

    }//end for

    for (int i(0); i < (int)vertices_.size(); ++i)
    {
      Vec3 vn = vertices_[i];
      vn.Normalize();
      if (vn * vertexNormals_[i] < 0)
      {
        vertexNormals_[i] = -vertexNormals_[i];
      }
    }


    delete[] pFacesAtVertex;

  }//end CalcVertexNormals

  MeshVertexIter Mesh3D::meshVertexBegin()
  {
    return vertices_.begin();
  }

  MeshVertexIter Mesh3D::meshVertexEnd()
  {
    return vertices_.end();
  }

  FaceIter Mesh3D::begin()
  {
    return faces_.begin();
  }

  FaceIter Mesh3D::end()
  {
    return faces_.end();
  }

  ConstFaceIter Mesh3D::begin() const
  {
    return faces_.begin();
  }

  ConstFaceIter Mesh3D::end() const
  {
    return faces_.end();
  }

  VECTOR3 Mesh3D::TransformModelWorldSingle(const VECTOR3 &vVec)
  {
    VECTOR3 vWorld;
    vWorld = transform_*vVec;
    vWorld += com_;
    return vWorld;
  }

  VECTOR3 Mesh3D::TransfromWorldModelSingle(const VECTOR3 &vVec)
  {
    MATRIX3X3 mrotMat = transform_.GetTransposedMatrix();
    VECTOR3 vModel;
    vModel = vVec - com_;
    return mrotMat*vModel;
  }

  void Mesh3D::TransformModelWorld()
  {
    for (auto &v : vertices_)
    {
      v = transform_ * v;
      v += com_;
    }
  }



  void Mesh3D::buildIndexArrays(void)
  {
    if (indices_ != nullptr)
    {
      return;
    }

    if (isTextured())
    {
      buildIndexArraysUV();
    }
    else if (!vertexNormals_.empty())
    {
      buildIndexArraysN();
    }

  }//end buildIndexArrays

  void Mesh3D::buildIndexArraysUV()
  {
    //allocate memory for the index array
    indices_ = new unsigned int[3 * this->faces_.size()];
    texIndices_ = new unsigned int[3 * this->faces_.size()];

    for (unsigned i(0); i < faces_.size(); ++i)
    {
      for (unsigned j(0); j < 3; ++j)
      {
        indices_[i * 3 + j] = faces_[i].m_VertIndices[j];
        texIndices_[i * 3 + j] = faces_[i].m_TexIndices[j];
        std::cout << "face: " << i << " index: " << faces_[i].m_VertIndices[j] << std::endl;
        std::cout << "texFace: " << i << " index: " << faces_[i].m_TexIndices[j] << std::endl;
      }//end for
      std::cout << "-----------" << std::endl;
    }//end for
  }

  void Mesh3D::buildIndexArraysN()
  {
    //allocate memory for the index array
    indices_ = new unsigned int[3 * this->faces_.size()];
    for (unsigned i(0); i < faces_.size(); ++i)
    {
      for (unsigned j(0); j < 3; ++j)
      {
        indices_[i * 3 + j] = faces_[i].m_VertIndices[j];
        std::cout << "face: " << i << "index: " << faces_[i].m_VertIndices[j] << std::endl;
      }//end for
    }//end for
  }

  void Mesh3D::calcRawVertexNormals()
  {

    using namespace std;

    //correctely size the vectors
    vector<vector<int>> pFacesAtVertex;
    pFacesAtVertex.reserve(rawVertices_.size());
    for (unsigned i(0); i < rawVertices_.size(); ++i)
      pFacesAtVertex.push_back(std::vector<int>());

    std::vector<Vec3> pNormals;
    std::vector<Vec3> tempNormals;
    pNormals.clear();
    vertexNormals_.clear();

    //calculate the face normals in a
    //first loop
    for (unsigned i(0); i < faces_.size(); ++i)
    {
      //get the vertex indices of the face
      int vi0 = faces_[i][0];
      int vi1 = faces_[i][1];
      int vi2 = faces_[i][2];

      //remember the face index
      pFacesAtVertex[vi0].push_back(i);
      pFacesAtVertex[vi1].push_back(i);
      pFacesAtVertex[vi2].push_back(i);

      Vec3 v0 = rawVertices_[vi0];
      Vec3 v1 = rawVertices_[vi1];
      Vec3 v2 = rawVertices_[vi2];

      //create 2 vectors, the face normal will be
      //perpendicular to those
      Vec3 vV1 = Vec3::createVector(v0, v2);
      Vec3 vV2 = Vec3::createVector(v2, v1);

      ////Calculate and assign the face normal		
      pNormals.push_back(Vec3::Cross(vV1, vV2));

    }//end for

    //in this 2nd loop calculate the vertex normals
    for (int i(0); i < (int)rawVertices_.size(); ++i)
    {

      Vec3 vSum(0, 0, 0);
      int count = 0;

      for (int j(0); j < (int)pFacesAtVertex[i].size(); ++j)
      {
        vSum += pNormals[pFacesAtVertex[i][j]];
        count++;
      }//end for

      //divide by the number of neighboring face
      //and normalize
      vSum /= Real(count);
      vSum.Normalize();
      tempNormals.push_back(vSum);

    }//end for

    for (auto &f : faces_)
    {

      for (int i(0); i < 3; ++i)
      {
        int i_v = f.m_VertIndices[i];
        vertexNormals_.push_back(tempNormals[i_v]);
      }
    }

  }//end CalcVertexNormals

  void Mesh3D::prepareNonIndexedRendering()
  {
    std::vector<Vec3> temp;
    orderedTexCoords_.clear();
    for (auto &f : faces_)
    {
      int i;
      for (i = 0; i < 3; ++i)
      {
        int i_t = f.m_TexIndices[i];
        int i_v = f.m_VertIndices[i];
        temp.push_back(vertices_[i_v]);
        orderedTexCoords_.push_back(texCoords_[i_t]);
      }
    }
    rawVertices_ = vertices_;
    vertices_ = temp;
  }

  // One has to choose the max(vertices.size(),texCoords.size())
  // and fill the smaller array with the required data
  void Mesh3D::prepareIndexArrays()
  {

    if (vertices_.size() >= texCoords_.size())
    {
      std::cout << "v>=vt" << std::endl;
      orderedTexCoords_ = std::vector<Vec2>(vertices_.size());
      for (auto &f : faces_)
      {
        int i;
        for (i = 0; i < 3; ++i)
        {
          int i_t = f.m_TexIndices[i];
          int i_v = f.m_VertIndices[i];

          orderedTexCoords_[i_v] = texCoords_[i_t];
        }
      }
    }
    else
    {
      orderedTexCoords_ = texCoords_;
      std::vector<Vec3> temp(texCoords_.size());

      for (auto &f : faces_)
      {
        int i;
        for (i = 0; i < 3; ++i)
        {
          int i_t = f.m_TexIndices[i];
          int i_v = f.m_VertIndices[i];
          temp[i_t] = vertices_[i_v];
          f.m_VertIndices[i] = i_t;
        }
      }
      vertices_ = temp;
      setNumVerts(vertices_.size());
    }

    //    orderedTexCoords_ = std::vector<Vec2>(texCoords_.size());
    //    for (auto &f : faces_)
    //    {
    //      int i;
    //      for (i = 0; i < 3; ++i)
    //      {
    //        int i_t = f.m_TexIndices[i]; 
    //        int i_v = f.m_VertIndices[i];
    //
    //        orderedTexCoords_[i_v] = texCoords_[i_t]; 
    //      } 
    //    }

    std::cout << "tex coord size: " << orderedTexCoords_.size() << std::endl;
    for (auto &i : orderedTexCoords_)
    {
      std::cout << "tex coord index: " << i << std::endl;
    }

  }

  void Mesh3D::generateBoundingBox()
  {
    std::vector<VECTOR3> Vec3Array;
    for (unsigned int i = 0; i < vertices_.size(); i++)
    {
      Vec3Array.push_back(vertices_[i]);
    }//end for
    box_.initBox(Vec3Array);
  }

}
