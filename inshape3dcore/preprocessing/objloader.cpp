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

ObjLoader::ObjLoader(void)
{

	uv_ = false;	

}//end constructor

ObjLoader::~ObjLoader(void)
{

}//end deconstructor

void ObjLoader::readModelFromFile(Model3D *pModel,const char *strFileName)
{

  ifstream in(strFileName);

  char strLine[256];
  string first;

  model_ = pModel;
  Mesh3D mesh;

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
      readVertex(in,strLine);
    //case: TexCoord
    else if(first == string("vt"))
    {
      readTexCoord(in, strLine);
      uv_ = true;
    }
    //case: Face
    else if(first == string("f"))
    {
      if(!uv_)
      {
        readFace(in, strLine);
      }
      else
        readFaceTex(in, strLine);
    }
    //case: Object groupd
    else if(first == string("o"))
    {
      std::cerr<<"Found Mesh(es) in non-supported OBJ object format. Please use the OBJ group format."<<std::endl;
      exit(0);
    }
    //case: Object groupd
    else if(first == string("g"))
    {
      in.getline(strLine,256);
    }        
    //default
    else
      in.getline(strLine,256);
        
  }//end while

  //cout <<"Number of vertices: "<<m_pVertices.size()<<endl;
  //cout <<"Number of faces: "<<m_pFaces.size()<<endl;

  //assign number of vertices
  mesh.vertices_.Resize(vertices_.size());
  mesh.numVerts_=vertices_.size();
  mesh.numFaces_=faces_.size();
  mesh.numTexCoords_=texCoords_.size();
  mesh.texCoords_.Resize(texCoords_.size());
  mesh.faces_.Resize(faces_.size());
  for(unsigned int i=0;i<vertices_.size();i++)
  {
    mesh.vertices_[i]=vertices_[i];
  }

  for(unsigned int i=0;i<texCoords_.size();i++)
  {
    mesh.texCoords_[i] =texCoords_[i];
  }

  for(unsigned int i=0;i<faces_.size();i++)
  {
    mesh.faces_[i].InitFace(faces_[i].VertexIndex);
  }//end for

  mesh.calcVertexNormals();
  model_->meshes_.push_back(mesh);

  //add a dummy material
  tMaterialInfo info;
  char f[255]="COWLOFTH.bmp";
  strcpy(info.strFile,f);
  info.texureId = 0;
  pModel->AddMaterial(info);
  pModel->meshes_[0].setMaterialId(0);

}//end ReadModelFromFile

void ObjLoader::readMultiMeshFromFile(Model3D *pModel,const char *strFileName)
{

	ifstream in(strFileName);

	char strLine[256];
	string first;

	model_ = pModel;
	Mesh3D mesh;

	if(!in.is_open())
	{
		std::cerr<<"Unable to open file: "<<strFileName<<std::endl;
		exit(0);
	}

  //count the number of sub-meshes
  int subMeshes = 0;
  offset_ = 0;
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
    Mesh3D mesh;
    readSubMesh(in,&mesh);
    pModel->meshes_.push_back(mesh);
    offset_+=mesh.getNumVerts();
  }

}//end ReadMultiMeshFromFile

void ObjLoader::readSubMesh(ifstream &in, Mesh3D *pMesh)
{

	char strLine[256];
	while(!in.eof())
	{
		in>>type_;
		if(type_ == string("#"))
		{
			in.getline(strLine,256);
			continue;
		}
		//case: Vertex
		else if(type_ == string("v"))
			readVertices(in,strLine);
		//case: TexCoord
		//case: Face
		else if(type_ == string("f"))
		{
			readFaces(in, strLine);
      break;
		}
		//default
		else
			in.getline(strLine,256);
	}//end while

	//assign number of vertices
	pMesh->vertices_.Resize(vertices_.size());
	pMesh->numVerts_=vertices_.size();
	pMesh->numFaces_=faces_.size();
	pMesh->numTexCoords_=texCoords_.size();
	pMesh->texCoords_.Resize(texCoords_.size());
	pMesh->faces_.Resize(faces_.size());

	for(unsigned int i=0;i<vertices_.size();i++)
	{
		pMesh->vertices_[i]=vertices_[i];
	}

	for(unsigned int i=0;i<texCoords_.size();i++)
	{
		pMesh->texCoords_[i] =texCoords_[i];
	}

	for(unsigned int i=0;i<faces_.size();i++)
	{
		pMesh->faces_[i].InitFace(faces_[i].VertexIndex);
	}//end for

  //reset the vectors
  faces_.clear();
  texCoords_.clear();
  vertices_.clear();

}

void ObjLoader::readVertices(ifstream &in, char strLine[])
{

  while(!in.eof() && type_==string("v"))
  {
    readVertex(in,strLine);
    in >> type_;
  }
}

void ObjLoader::readFaces(ifstream &in, char strLine[])
{
  while(!in.eof() && type_==string("f"))
  {
    readFace(in,strLine);
    in >> type_;
  }
}

void ObjLoader::readVertex(ifstream &in, char strLine[])
{

	Vec3 vec;
	in >> vec.x;
	in >> vec.y;
	in >> vec.z;
        //vec.y=-vec.y;
	in.getline(strLine,256);
	vertices_.push_back(vec);

}//end ReadVertex

void ObjLoader::readFace(ifstream &in, char strLine[])
{

	tObjFace Face;

	for(int i = 0; i < 3; i++)
	{
		in >> Face.VertexIndex[i];
		Face.VertexIndex[i]-=(offset_+1);
	}

	in.getline(strLine, 256);
	faces_.push_back(Face);

}//end ReadFace

void ObjLoader::readTexCoord(ifstream &in, char strLine[])
{
	
	VECTOR2 vec;
	in >> vec.x;
	in >> vec.y;
	texCoords_.push_back(vec);
	//cout<<m_pTexCoords.size()<<" "<<vec.x<<" "<<vec.y<<endl;
	in.getline(strLine,256);

}//end ReadTexCoord

void ObjLoader::readFaceTex(ifstream &in, char strLine[])
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
	faces_.push_back(Face);

}//end ReadFaceTex

const VertArray& ObjLoader::getVertices() const
{

	return vertices_;

}//end GetVertices

const FaceArray& ObjLoader::getFaces() const
{

	return faces_;

}//end GetVertices


bool ObjLoader::hasUV(void) const
{
	return uv_;
}

const TexCoordArray& ObjLoader::getTexCoords(void) const
{
	return texCoords_;
}//end GetTexCoords

}
