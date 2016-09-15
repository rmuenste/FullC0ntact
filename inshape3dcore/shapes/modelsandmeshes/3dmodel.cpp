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

#include "3dmodel.h"
#include <aabb3.h>
#include <string>

namespace i3d {

  Model3D::Model3D(void)  
  {
    numMaterials_ = 0;
  }//end constructor

  Model3D::~Model3D(void)
  {
  }//end deconstructor

  Model3D::Model3D(const Model3D &pModel)
  {
    this->numMaterials_=pModel.numMaterials_;
    this->materials_= pModel.materials_;
    this->meshes_=pModel.meshes_;

    radius_ = pModel.radius_;
    box_   = pModel.box_;

  }

  void Model3D::generateBoundingBox()
  {
    int iVertsTotal = 0;
    for(unsigned int i = 0; i < meshes_.size();i++)
    {
      iVertsTotal+=meshes_[i].getNumVertices();
    }//end for

    std::vector<Vec3> Vec3Array(iVertsTotal);
    int iVert=0;
    for(unsigned int i = 0; i < meshes_.size();i++)
    {
      for(int j=0;j<meshes_[i].getNumVertices();j++)
      {

        Vec3Array[iVert++]=meshes_[i].vertices_[j];
      }
    }//end for

    box_.initBox(Vec3Array);

    /*	cout<<m_bdBox.m_Verts[0].x<<" "<<m_bdBox.m_Verts[0].y<<" "<<m_bdBox.m_Verts[0].z<<endl;
        cout<<m_bdBox.m_Verts[1].x<<" "<<m_bdBox.m_Verts[1].y<<" "<<m_bdBox.m_Verts[1].z<<endl;*/

  }

  void Model3D::addMaterial(tMaterialInfo& pMatInfo)
  {

    //add the new material
    this->materials_.push_back(pMatInfo);
    this->numMaterials_++;

  }//end AddMaterial

  // Outputs the most important data of the model in a structered way
  void Model3D::outputModelInfo(void)
  {
    int iVertsTotal = 0;
    for(unsigned int i = 0; i < meshes_.size();i++)
    {
      iVertsTotal+=meshes_[i].getNumVertices();
    }//end for
    std::cout<<"MODEL DATA INFO"<<std::endl;
    std::cout<<"--------------------------"<<std::endl;
    std::cout<<"Number of Subobjects: "<<this->meshes_.size()<<std::endl;
    std::cout<<"--------------------------"<<std::endl;
    std::cout<<"Object names: "<<std::endl;
    for(unsigned int i = 0; i < meshes_.size();i++)
    {
      std::cout<<"'"<<meshes_[i].fileName_<<"'"<<std::endl;
    }//end for
    std::cout<<"--------------------------"<<std::endl;
    std::cout<<"Number of Vertices total: "<<iVertsTotal<<std::endl;
    std::cout<<"--------------------------"<<std::endl;
    std::cout<<"Number of Materials: "<<this->numMaterials_<<std::endl;
    for(unsigned int i = 0; i < this->materials_.size();i++)
    {
      std::cout<<"'"<<materials_[i].strName<<"'"<<std::endl;
      std::cout<<"'"<<materials_[i].strFile<<"'"<<std::endl;
    }//end for
  }//end OutputModelInfo


  void Model3D::prepareIndexing()
  {
    //loop oover all the meshes and build their subobjects
    //for (auto &mesh : meshes_)
    //{
    //  if (mesh.isTextured())
    //    mesh.prepareIndexArrays();

    //  mesh.buildIndexArrays();
    //  mesh.calcVertexNormals();
    //}
  }

  void Model3D::prepareNonIndexedRendering()
  {
    //loop oover all the meshes and build their subobjects
    //for (auto &mesh : meshes_)
    //{
    //  mesh.prepareNonIndexedRendering();
    //}//end for
  }

  void Model3D::buildSmoothNormals()
  {
    //for (auto &mesh : meshes_)
    //  mesh.calcRawVertexNormals();
  }

  void Model3D::buildFakeVertexNormals()
  {
    //for (auto &mesh : meshes_)
    //  mesh.calcFakeVertexNormals();
  }

  void Model3D::buildVertexArrays(void)
  {
    //loop oover all the meshes and build their subobjects
    //for (auto &mesh : meshes_)
    //  mesh.buildIndexArrays();
  }//end BuildVertexArrays


  void Model3D::createFrom(std::vector<Vec3> &vVertices, std::vector<TriFace> &vFaces)
  {

    this->numMaterials_=0;
    Mesh3D mesh;
    mesh.textured_=false;
    mesh.valid_=true;
    mesh.matId_=-1;
    mesh.numFaces_=vFaces.size();
    mesh.numVerts_=vVertices.size();
    mesh.numTexCoords_=0;
    mesh.vertices_.reserve(mesh.numVerts_);
    mesh.faces_.reserve(mesh.numFaces_);

    std::vector<VECTOR3>::iterator vIter;
    std::vector<TriFace>::iterator tIter;

    int index=0;
    for(vIter=vVertices.begin();vIter!=vVertices.end();vIter++)
    {
      VECTOR3 vec=*vIter;
      mesh.vertices_.push_back(vec);
      index++;
    }//end for

    index=0;
    for(tIter=vFaces.begin();tIter!=vFaces.end();tIter++)
    {
      TriFace tri=*tIter;
      mesh.faces_.push_back(tri);
      index++;
    }//end for

    meshes_.push_back(mesh);

  }

  MeshIter Model3D::begin()
  {
    return meshes_.begin();
  }

  MeshIter Model3D::end()
  {
    return meshes_.end();
  }

  std::vector<Triangle3r> Model3D::genTriangleVector()
  {
    std::vector<Triangle3r> vTriangles;
    MeshIter mIter = begin();
    int count = 0;
    for(;mIter != end();mIter++)
    {
      Mesh3D &mesh = *mIter;
      std::vector<TriFace>::iterator faceIter;

      for(faceIter=mesh.faces_.begin();faceIter!=mesh.faces_.end();faceIter++)
      {
        TriFace tri=*faceIter;

        //We loop through all triangular faces of the
        // model. This variable will hold the current face
        Triangle3r tri3(mesh.getVertices()[tri[0]],mesh.getVertices()[tri[1]],mesh.getVertices()[tri[2]]);
        tri3.idx_ = count++;
        vTriangles.push_back(tri3);
      }//end for
    }//end for
    return vTriangles;
  }

}
