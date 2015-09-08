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

	com_=VECTOR3(0,0,0);
  transform_.SetIdentity();

  triangleAABBs_ = nullptr;

}//end constructor

Mesh3D::Mesh3D(char *strName)
{
	int i=0;
	do{
		fileName_[i]=strName[i];
	}while(strName[i++]!=0);

  indices_ = nullptr;

	com_=VECTOR3(0,0,0);

  transform_.SetIdentity();

  triangleAABBs_ = nullptr;

}//end constructor

Mesh3D::~Mesh3D(void)
{

	if(indices_)
		delete[] indices_;

  indices_ = nullptr;

  if (triangleAABBs_)
    delete[] triangleAABBs_;

  triangleAABBs_ = nullptr;

}//end deconstructor

Mesh3D::Mesh3D(const Mesh3D &pMesh)
{
  this->textured_=pMesh.textured_;
  this->valid_     =pMesh.valid_;
  this->matId_        =pMesh.matId_;
  this->numFaces_  =pMesh.numFaces_;
  this->numTexCoords_=pMesh.numTexCoords_;
  this->numVerts_  =pMesh.numVerts_;
  this->faces_     =pMesh.faces_;
  this->indices_   =pMesh.indices_;
  this->texCoords_   =pMesh.texCoords_;
  this->vertexNormals_=pMesh.vertexNormals_;
  this->vertices_  =pMesh.vertices_;
	this->box_      =pMesh.box_;
  
  fileName_ = pMesh.fileName_;

  
//   while(pMesh.m_strName[i]!=0)
//   {
// 	  m_strName[i]=pMesh.m_strName[i];
// 	  i++;
//   };

  if(indices_ != nullptr)
  {
	int numIndices=3*this->faces_.Size();
	indices_ = new unsigned int[numIndices];
  memcpy(indices_,pMesh.indices_,numIndices);
  }

  triangleAABBs_ = nullptr;

  com_=pMesh.com_;
  transform_=pMesh.transform_;

}

void Mesh3D::generateTriangleBoundingBoxes()
{
  if (triangleAABBs_ == nullptr)
    triangleAABBs_ = new AABB3r[numFaces_];

  for (int i = 0; i < (int)faces_.Size(); i++)
  {
    int vi0 = m_pFaces[i][0];

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



    //void init(const Vector3<T> &minVec, const Vector3<T> &maxVec);
    //triangleAABBs_[i].

  }

}

void Mesh3D::calcVertexNormals()
{
	using namespace std;
	//correctely size the vectors
	vector<int>* pFacesAtVertex = new vector<int>[vertices_.Size()];
	Normal3Array pNormals;
	pNormals.Resize(faces_.Size());
	vertexNormals_.Resize(vertices_.Size());

	//calculate the face normals in a
	//first loop
	for(int i = 0; i < (int)faces_.Size(); i++)
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
		pNormals[i] = VECTOR3::Cross(vV1, vV2);
		
	}//end for

	//in this 2nd loop calculate the vertex normals
	for(int i = 0; i < (int)vertices_.Size(); i++)
	{

		VECTOR3 vSum(0,0,0);
		int count = 0;

		for(int j = 0; j < (int)pFacesAtVertex[i].size(); j++)
		{
				vSum+=pNormals[pFacesAtVertex[i][j]];
				count++;
		}//end for
		
		//divide by the number of neighboring face
		//and normalize
		vSum/=count;
		vSum.Normalize();
		vertexNormals_[i] =vSum;//*-1.0;

	}//end for

	delete[] pFacesAtVertex;

}//end CalcVertexNormals




	//for(int i = 0; i < m_pNormals.Size(); i++)
	//{
	//	glColor3f(1,0,0);
	//	VECTOR3 cog(0,0,0);
	//	cog = m_pVertices[i];
	//	glBegin(GL_LINES);	
	//	glVertex3f(cog.x, cog.y, cog.z);
	//	glVertex3f(cog.x + m_pNormals[i].x, cog.y + m_pNormals[i].y, cog.z + m_pNormals[i].z);
	//	glEnd();
	//}//end for


//}//end RenderModel

//FaceIter C3DMesh::begin()
//{
//  return FaceIter(&m_pFaces[0]);
//}
//FaceIter C3DMesh::end()
//{
//  return FaceIter(m_pFaces+m_pFaces.Size());
//}


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
	VECTOR3 vWorld;
	for(int i=0;i<vertices_.Size();i++)
	{
		vWorld = vertices_[i];
		vWorld = transform_*vWorld;
		vWorld += com_;
		vertices_[i] = vWorld;
	}
}


void Mesh3D::buildVertexArrays(void)
{
	//allocate memory for the index array
	this->indices_ = new unsigned int[3*this->faces_.Size()];
	for(int i=0;i<numFaces_;i++)
	{
		for(int j=0;j<3;j++)
		{
			indices_[i*3+j]=faces_[i][j];
//			cout<<" Index "<<i*3+j<<" vertex "<<m_pIndices[i*3+j];	
		}//end for
//		cout<<endl;
//		cout<<m_pFaces[i][0]<<" "<<m_pFaces[i][1]<<" "<<m_pFaces[i][2]<<endl;
	}//end for
}//end BuildVertexArrays

void Mesh3D::generateBoundingBox()
{
	CDynamicArray<VECTOR3> Vec3Array(m_pVertices.Size());
	for(int i = 0; i < m_pVertices.Size();i++)
	{
		Vec3Array[i]=vertices_[i];
	}//end for
	box_.initBox(Vec3Array);
/*	std::cout<<m_bdBox.m_Verts[0].x<<" "<<m_bdBox.m_Verts[0].y<<" "<<m_bdBox.m_Verts[0].z<<std::endl;
	std::cout<<m_bdBox.m_Verts[1].x<<" "<<m_bdBox.m_Verts[1].y<<" "<<m_bdBox.m_Verts[1].z<<std::endl;*/
}

}
