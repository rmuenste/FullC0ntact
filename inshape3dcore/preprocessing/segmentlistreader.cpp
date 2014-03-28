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

#include "segmentlistreader.h"
#include "3dmodel.h"
#include <fstream>
#include <sstream>
#include <string.h>

namespace i3d {

CSegmentListReader::CSegmentListReader(void)
{

}//end constructor

CSegmentListReader::~CSegmentListReader(void)
{

}//end deconstructor

void CSegmentListReader::ReadModelFromFile(ParamLiner *pLine,const char *strFileName)
{

  ifstream in(strFileName);

  char strLine[256];
  string first;

  m_pLine = pLine;

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
    {
      ReadVertex(in,strLine);
    }
    //case: Face
    else if(first == string("f"))
    {
      ReadFace(in, strLine);
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

  cout <<"Number of vertices: "<<m_pVertices.size()<<endl;
  cout <<"Number of faces: "<<m_pFaces.size()<<endl;

  //assign number of vertices
  for(unsigned int i=0;i<m_pVertices.size();i++)
  {
    pLine->vertices_.push_back(m_pVertices[i]);
  }

  int j,k;
  for(unsigned int i=0;i<m_pFaces.size();i++)
  {
    int i0=m_pFaces[i].VertexIndex[0];
    int i1=m_pFaces[i].VertexIndex[1];
    pLine->segments_.push_back(CSegment3<Real>(m_pVertices[i0],m_pVertices[i1]));
    pLine->faces_.push_back( std::pair<int,int>(i0,i1) );
  }//end for
  
}//end ReadModelFromFile

void CSegmentListReader::ReadVertices(ifstream &in, char strLine[])
{

  while(!in.eof() && type==string("v"))
  {
    ReadVertex(in,strLine);
    in >> type;
  }
}

void CSegmentListReader::ReadFaces(ifstream &in, char strLine[])
{
  while(!in.eof() && type==string("f"))
  {
    ReadFace(in,strLine);
    in >> type;
  }
}

void CSegmentListReader::ReadVertex(ifstream &in, char strLine[])
{
  if(in.eof())
    return;
  
	CVector3f vec;
	in >> vec.x;
	in >> vec.y;
	in >> vec.z;
        //vec.y=-vec.y;
	in.getline(strLine,256);
	m_pVertices.push_back(vec);

}//end ReadVertex

void CSegmentListReader::ReadFace(ifstream &in, char strLine[])
{

	tObjFace Face;
  if(in.eof())
    return;
  
	for(int i = 0; i < 2; i++)
	{
    in >> Face.VertexIndex[i];
    Face.VertexIndex[i]-=1;
	}

	in.getline(strLine, 256);
	m_pFaces.push_back(Face);

}//end ReadFace

const VertArray& CSegmentListReader::GetVertices() const
{

	return m_pVertices;

}//end GetVertices

const FaceArray& CSegmentListReader::GetFaces() const
{

	return m_pFaces;

}//end GetVertices

}
