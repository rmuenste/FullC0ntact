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

C3DMesh::C3DMesh(void)
{
	m_bValid = false;
	this->m_pIndices=NULL;

	m_vOrigin=VECTOR3(0,0,0);
  m_matTransform.SetIdentity();


}//end constructor

C3DMesh::C3DMesh(char *strName)
{
	int i=0;
	do{
		m_strName[i]=strName[i];
	}while(strName[i++]!=0);
	this->m_pIndices=NULL;

	m_vOrigin=VECTOR3(0,0,0);

  m_matTransform.SetIdentity();

}//end constructor

C3DMesh::~C3DMesh(void)
{

	if(m_pIndices)
		delete[] m_pIndices;

	m_pIndices=NULL;

}//end deconstructor

C3DMesh::C3DMesh(const C3DMesh &pMesh)
{
  this->m_bIsTextured=pMesh.m_bIsTextured;
  this->m_bValid     =pMesh.m_bValid;
  this->m_iID        =pMesh.m_iID;
  this->m_iNumFaces  =pMesh.m_iNumFaces;
  this->m_iNumTCoords=pMesh.m_iNumTCoords;
  this->m_iNumVerts  =pMesh.m_iNumVerts;
  this->m_pFaces     =pMesh.m_pFaces;
  this->m_pIndices   =pMesh.m_pIndices;
  this->m_pTCoords   =pMesh.m_pTCoords;
  this->m_pVertexNormals=pMesh.m_pVertexNormals;
  this->m_pVertices  =pMesh.m_pVertices;
	this->m_bdBox      =pMesh.m_bdBox;
  
  int iSize = strlen(pMesh.m_strName);
  if(iSize > 0)
    strncpy(m_strName,pMesh.m_strName,iSize);
  
//   while(pMesh.m_strName[i]!=0)
//   {
// 	  m_strName[i]=pMesh.m_strName[i];
// 	  i++;
//   };

  if(m_pIndices != NULL)
  {
	int numIndices=3*this->m_pFaces.Size();
	m_pIndices = new unsigned int[numIndices];
  memcpy(m_pIndices,pMesh.m_pIndices,numIndices);
  }

  m_vOrigin=pMesh.m_vOrigin;
  m_matTransform=pMesh.m_matTransform;

}

void C3DMesh::CalcVertexNormals()
{
	using namespace std;
	//correctely size the vectors
	vector<int>* pFacesAtVertex = new vector<int>[m_pVertices.Size()];
	Normal3Array pNormals;
	pNormals.Resize(m_pFaces.Size());
	m_pVertexNormals.Resize(m_pVertices.Size());

	//calculate the face normals in a
	//first loop
	for(int i = 0; i < (int)m_pFaces.Size(); i++)
	{
		//get the vertex indices of the face
		int vi0 = m_pFaces[i][0];
		int vi1 = m_pFaces[i][1];
		int vi2 = m_pFaces[i][2];

		//remember the face index
		pFacesAtVertex[vi0].push_back(i);
		pFacesAtVertex[vi1].push_back(i);
		pFacesAtVertex[vi2].push_back(i);

		VECTOR3 v0 = m_pVertices[vi0];
		VECTOR3 v1 = m_pVertices[vi1];
		VECTOR3 v2 = m_pVertices[vi2];

		//create 2 vectors, the face normal will be
		//perpendicular to those
		VECTOR3 vV1 = VECTOR3::createVector(v0, v2);
		VECTOR3 vV2 = VECTOR3::createVector(v2, v1);

		////Calculate and assign the face normal		
		pNormals[i] = VECTOR3::Cross(vV1, vV2);
		
	}//end for

	//in this 2nd loop calculate the vertex normals
	for(int i = 0; i < (int)m_pVertices.Size(); i++)
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
		m_pVertexNormals[i] =vSum;//*-1.0;

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


MeshVertexIter C3DMesh::MeshVertexBegin()
{
  return m_pVertices.begin();
}

MeshVertexIter C3DMesh::MeshVertexEnd()
{
  return m_pVertices.end();
}

FaceIter C3DMesh::begin()
{
  return m_pFaces.begin();
}

FaceIter C3DMesh::end()
{
  return m_pFaces.end();
}

ConstFaceIter C3DMesh::begin() const
{
  return m_pFaces.begin();
}

ConstFaceIter C3DMesh::end() const
{
  return m_pFaces.end();
}

VECTOR3 C3DMesh::TransformModelWorldSingle(const VECTOR3 &vVec)
{
	VECTOR3 vWorld;
	vWorld = m_matTransform*vVec;
	vWorld += m_vOrigin;
	return vWorld;
}

VECTOR3 C3DMesh::TransfromWorldModelSingle(const VECTOR3 &vVec)
{
  MATRIX3X3 mrotMat = m_matTransform.GetTransposedMatrix();
	VECTOR3 vModel; 
	vModel = vVec - m_vOrigin;
	return mrotMat*vModel;
}

void C3DMesh::TransformModelWorld()
{
	VECTOR3 vWorld;
	for(int i=0;i<m_pVertices.Size();i++)
	{
		vWorld = m_pVertices[i];
		vWorld = m_matTransform*vWorld;
		vWorld += m_vOrigin;
		m_pVertices[i] = vWorld;
	}
}


void C3DMesh::BuildVertexArrays(void)
{
	//allocate memory for the index array
	this->m_pIndices = new unsigned int[3*this->m_pFaces.Size()];
	for(int i=0;i<m_iNumFaces;i++)
	{
		for(int j=0;j<3;j++)
		{
			m_pIndices[i*3+j]=m_pFaces[i][j];
//			cout<<" Index "<<i*3+j<<" vertex "<<m_pIndices[i*3+j];	
		}//end for
//		cout<<endl;
//		cout<<m_pFaces[i][0]<<" "<<m_pFaces[i][1]<<" "<<m_pFaces[i][2]<<endl;
	}//end for
}//end BuildVertexArrays

void C3DMesh::GenerateBoundingBox()
{
	CDynamicArray<VECTOR3> Vec3Array(m_pVertices.Size());
	for(unsigned int i = 0; i < m_pVertices.Size();i++)
	{
		Vec3Array[i]=m_pVertices[i];
	}//end for
	m_bdBox.initBox(Vec3Array);
/*	std::cout<<m_bdBox.m_Verts[0].x<<" "<<m_bdBox.m_Verts[0].y<<" "<<m_bdBox.m_Verts[0].z<<std::endl;
	std::cout<<m_bdBox.m_Verts[1].x<<" "<<m_bdBox.m_Verts[1].y<<" "<<m_bdBox.m_Verts[1].z<<std::endl;*/
}

}