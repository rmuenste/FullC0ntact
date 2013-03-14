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

#include "objloader.h"
#include "3dmodel.h"
#include <fstream>
#include <sstream>
#include <string.h>

namespace i3d {

CObjLoader::CObjLoader(void)
{

	m_bUV = false;	

}//end constructor

CObjLoader::~CObjLoader(void)
{

}//end deconstructor

void CObjLoader::ReadModelFromFile(C3DModel *pModel,const char *strFileName)
{

	ifstream in(strFileName);

	char strLine[256];
	string first;

	m_pModel = pModel;
	C3DMesh mesh;

	if(!in.is_open())
	{
		std::cerr<<"Unable to open file: "<<strFileName<<std::endl;
		exit(0);
	}

	while(!in.eof())
	{
		
		in>>first;
		
		if(first == string("#"))
		{
			in.getline(strLine,256);
			continue;
		}
		else if(first == string(""))
		{
			in.getline(strLine,256);
			continue;
		}
		//case: Vertex
		else if(first == string("v"))
			ReadVertex(in,strLine);
		//case: TexCoord
		else if(first == string("vt"))
		{
			ReadTexCoord(in, strLine);
			m_bUV = true;
		}
		//case: Face
		else if(first == string("f"))
		{
			if(!m_bUV)
			{
				ReadFace(in, strLine);
			}
			else
				ReadFaceTex(in, strLine);
		}
		//default
		else
			in.getline(strLine,256);
		

		
	}//end while

	//cout <<"Number of vertices: "<<m_pVertices.size()<<endl;
	//cout <<"Number of faces: "<<m_pFaces.size()<<endl;

	//assign number of vertices
	mesh.m_pVertices.Resize(m_pVertices.size());
	mesh.m_iNumVerts=m_pVertices.size();
	mesh.m_iNumFaces=m_pFaces.size();
	mesh.m_iNumTCoords=m_pTexCoords.size();
	mesh.m_pTCoords.Resize(m_pTexCoords.size());
	mesh.m_pFaces.Resize(m_pFaces.size());
	for(unsigned int i=0;i<m_pVertices.size();i++)
	{
		mesh.m_pVertices[i]=m_pVertices[i];
	}

	for(unsigned int i=0;i<m_pTexCoords.size();i++)
	{
		mesh.m_pTCoords[i] =m_pTexCoords[i];
	}

	for(unsigned int i=0;i<m_pFaces.size();i++)
	{
		mesh.m_pFaces[i].InitFace(m_pFaces[i].VertexIndex);
	}//end for

	mesh.CalcVertexNormals();
	m_pModel->m_vMeshes.push_back(mesh);

	//add a dummy material
	tMaterialInfo info;
	char f[255]="COWLOFTH.bmp";
	strcpy(info.strFile,f);
	info.texureId = 0;
	pModel->AddMaterial(info);
	pModel->m_vMeshes[0].SetMaterialID(0);


}//end ReadModelFromFile

void CObjLoader::ReadMultiMeshFromFile(C3DModel *pModel,const char *strFileName)
{

	ifstream in(strFileName);

	char strLine[256];
	string first;

	m_pModel = pModel;
	C3DMesh mesh;

	if(!in.is_open())
	{
		std::cerr<<"Unable to open file: "<<strFileName<<std::endl;
		exit(0);
	}

  //count the number of sub-meshes
  int subMeshes = 0;
  m_iOffset = 0;
	while(!in.eof())
	{
		in>>first;
		if(first == string("g"))
		{
      subMeshes++;
			in.getline(strLine,256);
		}
    else
			in.getline(strLine,256);
	}//end while

  in.clear();
  in.seekg(0,ios::beg);

  for(int i=0;i< subMeshes;i++)
  {
    C3DMesh mesh;
    ReadSubMesh(in,&mesh);
    pModel->m_vMeshes.push_back(mesh);
    m_iOffset+=mesh.GetNumVerts();
  }

}//end ReadMultiMeshFromFile

void CObjLoader::ReadSubMesh(ifstream &in, C3DMesh *pMesh)
{

	char strLine[256];
	while(!in.eof())
	{
		in>>type;
		if(type == string("#"))
		{
			in.getline(strLine,256);
			continue;
		}
		//case: Vertex
		else if(type == string("v"))
			ReadVertices(in,strLine);
		//case: TexCoord
		//case: Face
		else if(type == string("f"))
		{
			ReadFaces(in, strLine);
      break;
		}
		//default
		else
			in.getline(strLine,256);
	}//end while

	//assign number of vertices
	pMesh->m_pVertices.Resize(m_pVertices.size());
	pMesh->m_iNumVerts=m_pVertices.size();
	pMesh->m_iNumFaces=m_pFaces.size();
	pMesh->m_iNumTCoords=m_pTexCoords.size();
	pMesh->m_pTCoords.Resize(m_pTexCoords.size());
	pMesh->m_pFaces.Resize(m_pFaces.size());

	for(unsigned int i=0;i<m_pVertices.size();i++)
	{
		pMesh->m_pVertices[i]=m_pVertices[i];
	}

	for(unsigned int i=0;i<m_pTexCoords.size();i++)
	{
		pMesh->m_pTCoords[i] =m_pTexCoords[i];
	}

	for(unsigned int i=0;i<m_pFaces.size();i++)
	{
		pMesh->m_pFaces[i].InitFace(m_pFaces[i].VertexIndex);
	}//end for

  //reset the vectors
  m_pFaces.clear();
  m_pTexCoords.clear();
  m_pVertices.clear();

}

void CObjLoader::ReadVertices(ifstream &in, char strLine[])
{

  while(!in.eof() && type==string("v"))
  {
    ReadVertex(in,strLine);
    in >> type;
  }
}

void CObjLoader::ReadFaces(ifstream &in, char strLine[])
{
  while(!in.eof() && type==string("f"))
  {
    ReadFace(in,strLine);
    in >> type;
  }
}

void CObjLoader::ReadVertex(ifstream &in, char strLine[])
{

	CVector3f vec;
	in >> vec.x;
	in >> vec.y;
	in >> vec.z;
        //vec.y=-vec.y;
	in.getline(strLine,256);
	m_pVertices.push_back(vec);

}//end ReadVertex

void CObjLoader::ReadFace(ifstream &in, char strLine[])
{

	tObjFace Face;

	for(int i = 0; i < 3; i++)
	{
		in >> Face.VertexIndex[i];
		Face.VertexIndex[i]-=(m_iOffset+1);
	}

	in.getline(strLine, 256);
	m_pFaces.push_back(Face);

}//end ReadFace

void CObjLoader::ReadTexCoord(ifstream &in, char strLine[])
{
	
	VECTOR2 vec;
	in >> vec.x;
	in >> vec.y;
	m_pTexCoords.push_back(vec);
	//cout<<m_pTexCoords.size()<<" "<<vec.x<<" "<<vec.y<<endl;
	in.getline(strLine,256);

}//end ReadTexCoord

void CObjLoader::ReadFaceTex(ifstream &in, char strLine[])
{
	tObjFace Face;

	string s;

	basic_string<char> vertIndex;
	basic_string<char> texIndex;
	int vi;
	int ti;

	for(int i = 0; i < 3; i++)
	{
		
		// Format for a face is vertexIndex/texture index vertexIndex/textureIndex vertexIndex/texture index 
		in >> s;
		
		// find separator 
		basic_string<char>::size_type index = s.find("/");

		// extract indices
		vertIndex = s.substr(0,index);
		texIndex = s.substr(index+1,s.size()-1);

		// convert indices from string to int
		istringstream VertStream(vertIndex);
		istringstream TexStream(texIndex);

		VertStream >> vi;
		TexStream  >> ti;

		//assign the values to the face structure
		Face.VertexIndex[i] = vi-1;
		Face.TexIndex[i]    = ti-1;		
		
	}



	//go to next line
	in.getline(strLine, 256);
	m_pFaces.push_back(Face);

}//end ReadFaceTex

const VertArray& CObjLoader::GetVertices() const
{

	return m_pVertices;

}//end GetVertices

const FaceArray& CObjLoader::GetFaces() const
{

	return m_pFaces;

}//end GetVertices


bool CObjLoader::HasUV(void) const
{
	return m_bUV;
}

const TexCoordArray& CObjLoader::GetTexCoords(void) const
{
	return m_pTexCoords;
}//end GetTexCoords

}
