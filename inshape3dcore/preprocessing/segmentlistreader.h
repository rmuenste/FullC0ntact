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

#ifdef WIN32
#pragma once
#endif

#ifndef _SEGEMENTLISTREADER_H_
#define _SEGEMENTLISTREADER_H_


#include <vector>
#include <vector3.h>
#include <vector2.h>
#include <dynamicarray.h>
#include <objloader.h>
#include <paramline.h>

namespace i3d {

class C3DModel;
class C3DMesh;

using namespace std;

/**
* @brief A class that loads a list of line segments from an .obj files
*
* A class that loads .obj files and converts them into a C3DModel
* the internally used file format in this library
*
*/
class CSegmentListReader 
{
public:
	CSegmentListReader(void);
	~CSegmentListReader(void);

	/* reads the .obj file specified in strFileName */
	void ReadModelFromFile(CParamLiner *pLine,const char *strFileName);
	
	const VertArray& GetVertices() const;
	const FaceArray& GetFaces() const;
	const Vec3Array& GetNormals() const;

private:

	void ReadVertex(ifstream &in, char strLine[]);

	void ReadVertices(ifstream &in, char strLine[]);

  void ReadFaces(ifstream &in, char strLine[]);

	void ReadFace(ifstream &in, char strLine[]);

	/* private member variables */

  CParamLiner *m_pLine;
  
	VertArray m_pVertices;

	FaceArray m_pFaces;

  std::string type;

  int m_iOffset;

};

}

#endif
