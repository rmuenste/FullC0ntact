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

C3DModel::C3DModel(void)  
{
	m_iNumMaterials = 0;
}//end constructor

C3DModel::~C3DModel(void)
{
}//end deconstructor

C3DModel::C3DModel(const C3DModel &pModel)
{
  this->m_iNumMaterials=pModel.m_iNumMaterials;
  this->m_pMaterials= pModel.m_pMaterials;
  this->m_vMeshes=pModel.m_vMeshes;

	m_dRadius = pModel.m_dRadius;
	m_bdBox   = pModel.m_bdBox;

}

void C3DModel::GenerateBoundingBox()
{
	int iVertsTotal = 0;
	for(unsigned int i = 0; i < m_vMeshes.size();i++)
	{
		iVertsTotal+=m_vMeshes[i].NumVertices();
	}//end for

	CDynamicArray<VECTOR3> Vec3Array(iVertsTotal);
	int iVert=0;
	for(unsigned int i = 0; i < m_vMeshes.size();i++)
	{
		for(int j=0;j<m_vMeshes[i].NumVertices();j++)
		{
			
			Vec3Array[iVert++]=m_vMeshes[i].m_pVertices[j];
		}
	}//end for

	m_bdBox.initBox(Vec3Array);

/*	cout<<m_bdBox.m_Verts[0].x<<" "<<m_bdBox.m_Verts[0].y<<" "<<m_bdBox.m_Verts[0].z<<endl;
	cout<<m_bdBox.m_Verts[1].x<<" "<<m_bdBox.m_Verts[1].y<<" "<<m_bdBox.m_Verts[1].z<<endl;*/
	
}

void C3DModel::AddMaterial(tMaterialInfo& pMatInfo)
{

	//add the new material
	this->m_pMaterials.push_back(pMatInfo);
	this->m_iNumMaterials++;

}//end AddMaterial

// Outputs the most important data of the model in a structered way
void C3DModel::OutputModelInfo(void)
{
	int iVertsTotal = 0;
	for(unsigned int i = 0; i < m_vMeshes.size();i++)
	{
		iVertsTotal+=m_vMeshes[i].NumVertices();
	}//end for
	std::cout<<"MODEL DATA INFO"<<std::endl;
	std::cout<<"--------------------------"<<std::endl;
	std::cout<<"Number of Subobjects: "<<this->m_vMeshes.size()<<std::endl;
	std::cout<<"--------------------------"<<std::endl;
	std::cout<<"Object names: "<<std::endl;
	for(unsigned int i = 0; i < m_vMeshes.size();i++)
	{
		std::cout<<"'"<<m_vMeshes[i].m_strName<<"'"<<std::endl;
	}//end for
	std::cout<<"--------------------------"<<std::endl;
	std::cout<<"Number of Vertices total: "<<iVertsTotal<<std::endl;
	std::cout<<"--------------------------"<<std::endl;
	std::cout<<"Number of Materials: "<<this->m_iNumMaterials<<std::endl;
	for(unsigned int i = 0; i < this->m_pMaterials.size();i++)
	{
		std::cout<<"'"<<m_pMaterials[i].strName<<"'"<<std::endl;
		std::cout<<"'"<<m_pMaterials[i].strFile<<"'"<<std::endl;
	}//end for
}//end OutputModelInfo


void C3DModel::BuildVertexArrays(void)
{

	//loop oover all the meshes and build their subobjects
	for(unsigned int i = 0; i < m_vMeshes.size();i++)
	{
		m_vMeshes[i].BuildVertexArrays();
	}//end for

}//end BuildVertexArrays


void C3DModel::CreateFrom(std::vector<VECTOR3 > &vVertices, std::vector<TriFace> &vFaces)
{

  this->m_iNumMaterials=0;
  C3DMesh mesh;
  mesh.m_bIsTextured=false;
  mesh.m_bValid=true;
  mesh.m_iID=-1;
  mesh.m_iNumFaces=vFaces.size();
  mesh.m_iNumVerts=vVertices.size();
  mesh.m_iNumTCoords=0;
  mesh.m_pVertices.Resize(mesh.m_iNumVerts);
  mesh.m_pFaces.Resize(mesh.m_iNumFaces);

  std::vector<VECTOR3>::iterator vIter;
  std::vector<TriFace>::iterator tIter;

  int index=0;
  for(vIter=vVertices.begin();vIter!=vVertices.end();vIter++)
  {
	VECTOR3 vec=*vIter;
	mesh.m_pVertices[index]=vec;
	index++;
  }//end for

  index=0;
  for(tIter=vFaces.begin();tIter!=vFaces.end();tIter++)
  {
	TriFace tri=*tIter;
	mesh.m_pFaces[index]=tri;
	index++;
  }//end for

  m_vMeshes.push_back(mesh);

}

MeshIter C3DModel::begin()
{
  return m_vMeshes.begin();
}

MeshIter C3DModel::end()
{
  return m_vMeshes.end();
}

std::vector<Triangle3r> C3DModel::GenTriangleVector()
{
	std::vector<Triangle3r> vTriangles;
	MeshIter mIter = begin();
  int count = 0;
	for(;mIter != end();mIter++)
	{
		C3DMesh &mesh = *mIter;
		CDynamicArray<TriFace>::iterator faceIter;

		for(faceIter=mesh.m_pFaces.begin();faceIter!=mesh.m_pFaces.end();faceIter++)
		{
			TriFace tri=*faceIter;
      
			//We loop through all triangular faces of the
			// model. This variable will hold the current face
			Triangle3r tri3(mesh.GetVertices()[tri[0]],mesh.GetVertices()[tri[1]],mesh.GetVertices()[tri[2]]);
      tri3.idx_ = count++;
			vTriangles.push_back(tri3);
		}//end for
	}//end for
	return vTriangles;
}

}
