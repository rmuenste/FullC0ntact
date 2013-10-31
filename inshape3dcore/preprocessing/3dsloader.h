/***************************************************************************
 *   Copyright (C) 2009 by Raphael Münster   *
 *   raphael@   *
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

#ifndef _3DSLOADER_H_
#define _3DSLOADER_H_


#include <vector>
#include <stdio.h>
#include <stdlib.h>

#define MAIN3DS       0x4D4D
#define EDIT3DS       0x3D3D  // this is the start of the editor config
#define VERSION       0x0002  // the version number of the .3ds file

#define EDIT_MATERIAL 0xAFFF
#define EDIT_CONFIG1  0x0100
#define EDIT_CONFIG2  0x3E3D
#define EDIT_VIEW_P1  0x7012
#define EDIT_VIEW_P2  0x7011
#define EDIT_VIEW_P3  0x7020
#define EDIT_VIEW1    0x7001
#define EDIT_BACKGR   0x1200
#define EDIT_AMBIENT  0x2100
#define EDIT_OBJECT   0x4000

//>------ sub defines of MATERIAL
#define MATNAME       0xA000				// This holds the material name
#define MATDIFFUSE    0xA020				// This holds the color of the object/material
#define MATMAP        0xA200				// This is a header for a new material
#define MATMAPFILE    0xA300				// This holds the file name of the texture

#define OBJECT_MESH   0x4100				// This lets us know that we are reading a new object

//>------ sub defines of OBJECT_MESH
#define OBJECT_VERTICES     0x4110			// The objects vertices
#define OBJECT_FACES		0x4120			// The objects faces
#define OBJECT_MATERIAL		0x4130			// This is found if the object has a material, either texture map or color
#define OBJECT_UV			0x4140			// The UV texture coordinates


#define EDIT_UNKNW01  0x1100
#define EDIT_UNKNW02  0x1201
#define EDIT_UNKNW03  0x1300
#define EDIT_UNKNW04  0x1400
#define EDIT_UNKNW05  0x1420
#define EDIT_UNKNW06  0x1450
#define EDIT_UNKNW07  0x1500
#define EDIT_UNKNW08  0x2200
#define EDIT_UNKNW09  0x2201
#define EDIT_UNKNW10  0x2210
#define EDIT_UNKNW11  0x2300
#define EDIT_UNKNW12  0x2302 
#define EDIT_UNKNW13  0x3000
#define EDIT_UNKNW14  0xAFFF

namespace i3d {

///@cond HIDDEN_SYMBOLS  
// this structure represents a chunk
struct tChunk
{
	unsigned short int ID;					// The chunk's ID		
	unsigned int length;					// The length of the chunk
	unsigned int bytesRead;					// The amount of bytes read within that chunk
};
///@cond 

class C3DModel;
class C3DMesh;

// In a short amount of words, a chunk is defined this way:
// 2 bytes - Stores the chunk ID (OBJECT, MATERIAL, PRIMARY, etc...)
// 4 bytes - Stores the length of that chunk.  That way you know when that
//           chunk is done and there is a new chunk.
//
// Start by reading the first 2 bytes, that is the chunk ID, the read the next 4 bytes,
// the length (using fread()).  We can then check the ID, which should be PRIMARY, because
// it is the first chunk.
//
// The 3ds file is structured like this:
//
//      MAIN3DS  (0x4D4D)
//     |
//     +--EDIT3DS  (0x3D3D)
//     |  |
//     |  +--EDIT_MATERIAL (0xAFFF)
//     |  |  |
//     |  |  +--MAT_NAME01 (0xA000)  
//     |  |
//     |  +--EDIT_CONFIG1  (0x0100)
//     |  +--EDIT_CONFIG2  (0x3E3D) 
//     |  +--EDIT_VIEW_P1  (0x7012)
//     |  |  |
//     |  |  +--TOP            (0x0001)
//     |  |  +--BOTTOM         (0x0002)
//     |  |  +--LEFT           (0x0003)
//     |  |  +--RIGHT          (0x0004)
//     |  |  +--FRONT          (0x0005) 
//     |  |  +--BACK           (0x0006)
//     |  |  +--USER           (0x0007)
//     |  |  +--CAMERA         (0xFFFF)
//     |  |  +--LIGHT          (0x0009)
//     |  |  +--DISABLED       (0x0010)  
//     |  |  +--BOGUS          (0x0011)
//     |  |
//     |  +--EDIT_VIEW_P2  (0x7011)
//     |  |  |
//     |  |  +--TOP            (0x0001)
//     |  |  +--BOTTOM         (0x0002)
//     |  |  +--LEFT           (0x0003)
//     |  |  +--RIGHT          (0x0004)
//     |  |  +--FRONT          (0x0005) 
//     |  |  +--BACK           (0x0006)
//     |  |  +--USER           (0x0007)
//     |  |  +--CAMERA         (0xFFFF)
//     |  |  +--LIGHT          (0x0009)
//     |  |  +--DISABLED       (0x0010)  
//     |  |  +--BOGUS          (0x0011)
//     |  |
//     |  +--EDIT_VIEW_P3  (0x7020)
//     |  +--EDIT_VIEW1    (0x7001) 
//     |  +--EDIT_BACKGR   (0x1200) 
//     |  +--EDIT_AMBIENT  (0x2100)
//     |  +--EDIT_OBJECT   (0x4000)
//     |  |  |
//     |  |  +--OBJ_TRIMESH   (0x4100)      
//     |  |  |  |
//     |  |  |  +--TRI_VERTEXL          (0x4110)
//     |  |  |  +--TRI_VERTEXOPTIONS    (0x4111)
//     |  |  |  +--TRI_MAPPINGCOORS     (0x4140)
//     |  |  |  +--TRI_MAPPINGSTANDARD  (0x4170)
//     |  |  |  +--TRI_FACEL1           (0x4120)
//     |  |  |  |  |
//     |  |  |  |  +--TRI_SMOOTH            (0x4150)
//     |  |  |  |  +--TRI_MATERIAL          (0x4130)
//     |  |  |  |
//     |  |  |  +--TRI_LOCAL            (0x4160)
//     |  |  |  +--TRI_VISIBLE          (0x4165)
//     |  |  |
//     |  |  +--OBJ_LIGHT    (0x4600)
//     |  |  |  |
//     |  |  |  +--LIT_OFF              (0x4620)
//     |  |  |  +--LIT_SPOT             (0x4610)
//     |  |  |  +--LIT_UNKNWN01         (0x465A)
//     |  |  | 
//     |  |  +--OBJ_CAMERA   (0x4700)
//     |  |  |  |
//     |  |  |  +--CAM_UNKNWN01         (0x4710)
//     |  |  |  +--CAM_UNKNWN02         (0x4720)  
//     |  |  |
//     |  |  +--OBJ_UNKNWN01 (0x4710)
//     |  |  +--OBJ_UNKNWN02 (0x4720)
//     |  |
//     |  +--EDIT_UNKNW01  (0x1100)
//     |  +--EDIT_UNKNW02  (0x1201) 
//     |  +--EDIT_UNKNW03  (0x1300)
//     |  +--EDIT_UNKNW04  (0x1400)
//     |  +--EDIT_UNKNW05  (0x1420)
//     |  +--EDIT_UNKNW06  (0x1450)
//     |  +--EDIT_UNKNW07  (0x1500)
//     |  +--EDIT_UNKNW08  (0x2200)
//     |  +--EDIT_UNKNW09  (0x2201)
//     |  +--EDIT_UNKNW10  (0x2210)
//     |  +--EDIT_UNKNW11  (0x2300)
//     |  +--EDIT_UNKNW12  (0x2302)
//     |  +--EDIT_UNKNW13  (0x2000)
//     |  +--EDIT_UNKNW14  (0xAFFF)
//     |
//     +--KEYF3DS (0xB000)
//        |
//        +--KEYF_UNKNWN01 (0xB00A)
//        +--............. (0x7001) ( viewport, same as editor )
//        +--KEYF_FRAMES   (0xB008)
//        +--KEYF_UNKNWN02 (0xB009)
//        +--KEYF_OBJDES   (0xB002)
//           |
//           +--KEYF_OBJHIERARCH  (0xB010)
//           +--KEYF_OBJDUMMYNAME (0xB011)
//           +--KEYF_OBJUNKNWN01  (0xB013)
//           +--KEYF_OBJUNKNWN02  (0xB014)
//           +--KEYF_OBJUNKNWN03  (0xB015)  
//           +--KEYF_OBJPIVOT     (0xB020)  
//           +--KEYF_OBJUNKNWN04  (0xB021)  
//           +--KEYF_OBJUNKNWN05  (0xB022)  


using namespace std;

/**
* @brief A class that loads .3DS files
*
* A class that loads .3DS files and converts them into a C3DModel
* the internally used file format in this library
*
*/
class C3DSLoader 
{
public:
	C3DSLoader(void);

	~C3DSLoader(void);

	/* reads the .obj file specified in strFileName */
/**
* @brief Load a .3DS file
*
* If you export from Blender use these export settings:
* 'triangulate faces', forward axis:-Y, up axis: Z
* 'export objects as obj groups' is preferred
*/
	void ReadModelFromFile(C3DModel *pModel,const char *strFileName);
	void ReadModelFromFile(char *strFileName){};

private:
  /**
   * @brief Read the main chunk of a 3ds file
   * 
   * The chunk structure of a 3ds file is displayed above in the 
   * header file. The function initiates reading the different 
   * chunks in the depicted order
   * 
   */
  void ReadMainChunk(unsigned int iLength, unsigned int iPosition);

  // Read a simple chunk
  void ReadChunk(tChunk& pChunk);
  
  int ReadEditChunk(tChunk& pEditChunk);
  
  void ReadMaterialChunk(tChunk& pMaterialChunk);
  
  void ReadObjectChunk(tChunk& pObjectChunk,C3DMesh& pMesh);
  
  void ReadVertexChunk(tChunk& pVertexChunk,C3DMesh& pMesh);
  
  void ReadTrimeshChunk(tChunk& pTriMeshChunk,C3DMesh& pMesh);
  
  void ReadObjectMaterialChunk(tChunk& pObjectMaterialChunk,C3DMesh& pMesh);
  
  void ReadVertexIndexChunk(tChunk& pVertexIndexChunk,C3DMesh& pMesh);
  
  /**
   * @brief Read the texture coordinates
   * 
   * Read the texture coordinates
   */
  void ReadTexCoordChunk(tChunk& pTCoordChunk,C3DMesh& pMesh);
  int  GetString(char *pBuffer);

  C3DModel *m_pModel;
  FILE *m_FilePointer;
  int m_iBuffer[50000];  

};

}

#endif
