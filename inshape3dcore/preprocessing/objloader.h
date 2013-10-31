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

#ifndef _OBJLOADER_H_
#define _OBJLOADER_H_


#include <vector>
#include <vector3.h>
#include <vector2.h>
#include <dynamicarray.h>

namespace i3d {

class C3DModel;
class C3DMesh;

using namespace std;

///@cond HIDDEN_SYMBOLS
typedef struct
{

	int VertexIndex[3];
	int TexIndex[3];

}tObjFace;
///@cond 

typedef vector<VECTOR3> VertArray;
typedef vector<tObjFace>  FaceArray;
typedef vector<VECTOR2> TexCoordArray;
typedef CDynamicArray<VECTOR3> Vec3Array;
typedef CDynamicArray<VECTOR2> Vec2Array;

/**
* @brief A class that loads .obj files
*
* A class that loads .obj files and converts them into a C3DModel
* the internally used file format in this library
*
*/
class CObjLoader 
{
public:
	CObjLoader(void);
	~CObjLoader(void);

	/* reads the .obj file specified in strFileName */
	void ReadModelFromFile(C3DModel *pModel,const char *strFileName);


/**
* @brief Load a .obj file
*
* If you export from Blender use these export settings:
* 'triangulate faces', forward axis:-Y, up axis: Z
* 'triangulate faces', forward axis:-Z, up axis: Y
* 'export objects as obj groups' is preferred
*/
  void ReadMultiMeshFromFile(C3DModel *pModel,const char *strFileName);

  void ReadModelFromFile(char *strFileName){};

/**
  * @brief Returns the vertices of the reader's mesh structure
  * 
  * Returns the vertices of the reader's mesh structure
  */
  const VertArray& GetVertices() const;
  
/**
  * @brief Returns the faces of the reader's mesh structure
  * 
  * Returns the vertices of the reader's mesh structure
  */  
  const FaceArray& GetFaces() const;
  
/**
  * @brief Returns the normals of the reader's mesh structure
  * 
  * Returns the vertices of the reader's mesh structure
  */  
  const Vec3Array& GetNormals() const;

/**
  * @brief Returns the texture coordinates of the reader's mesh structure
  * 
  * Returns the texture coordinates of the reader's mesh structure
  */    
  const TexCoordArray& GetTexCoords(void) const;


  bool HasUV(void) const;

  private:

/**
 * @brief Reads the vertex section from the input stream
 * 
 * Reads the vertex section from the input stream
 */    
  void ReadVertex(ifstream &in, char strLine[]);

/**
 * @brief Reads the vertex section from the input stream
 * 
 * Reads the vertex section from the input stream
 */      
  void ReadVertices(ifstream &in, char strLine[]);

/**
 * @brief Reads the face section from the input stream
 * 
 * Reads the face section from the input stream
 */      
  void ReadFaces(ifstream &in, char strLine[]);

/**
 * @brief Reads a single face from the input stream
 * 
 * Reads a single face from the input stream
 */      
  void ReadFace(ifstream &in, char strLine[]);

/**
 * @brief Reads a single texture coordinate from the input stream
 * 
 * Reads a single texture coordinate from the input stream
 */      
  void ReadTexCoord(ifstream &in, char strLine[]);

/**
 * @brief Reads the texture coordinates for a face from the stream
 * 
 * Reads the texture coordinates for a face from the stream
 */      
  void ReadFaceTex(ifstream &in, char strLine[]);

/**
 * @brief Initiates reading of a sub mesh from the stream
 * 
 * Initiates reading of a sub mesh from the stream
 */      
  void ReadSubMesh(ifstream &in, C3DMesh *pMesh);

  /* private member variables */

  VertArray m_pVertices;

  TexCoordArray m_pTexCoords;

  FaceArray m_pFaces;

  bool m_bUV;

  C3DModel *m_pModel;

  std::string type;

  int m_iOffset;

};

}

#endif
