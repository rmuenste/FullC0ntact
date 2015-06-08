
/***************************************************************************
 *   Copyright (C) 2009 by Raphael Münster   *
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

#ifndef _3DMESH_H_
#define _3DMESH_H_

//===================================================
//					INCLUDES
//===================================================


#include <vector>
#include <cstdlib>
#include <vector3.h>
#include <vector2.h>
#include <triface.h>
#include <dynamicarray.h>
#include <aabb3.h>
#include <string.h>
#include <matrix3x3.h>

namespace i3d {

/**
	Typedefs for the model data
*/
typedef CDynamicArray<VECTOR3> Vertex3Array;
typedef CDynamicArray<VECTOR2> TCoordArray;
typedef CDynamicArray<VECTOR3> Normal3Array;
typedef CDynamicArray<TriFace>  TriFaceArray;
typedef CDynamicArray<TriFace>::iterator FaceIter;
typedef CDynamicArray<TriFace>::const_iterator ConstFaceIter;

typedef CDynamicArray<VECTOR3>::iterator MeshVertexIter;


/** \brief A C3DMesh is a class that holds actual mesh data like vertices and connectivity.
 *
 * A C3DMesh is a class that holds actual mesh data like vertices and connectivity. A C3DModel can be composed of one or more
 * C3DMeshes.
 */
class C3DMesh
{
public:

	/** \brief Standard constructor for the C3Dmesh class
	 *
	 */
	C3DMesh(void);

	/** 
	 * Copies a C3DMesh to another 
	 */	
	C3DMesh(const C3DMesh &pMesh);

	/** 
	 *
	 * A more extensive description of myProcedure().
	 * \param aParameter A brief description of aParameter.
	 */
	C3DMesh(char *strName);

	/** \brief Destructor.
	 *
	 */
	~C3DMesh(void);

	/** \brief Returns the array of vertices.
	 *
	 * \return Returns a Vertex3Array reference that stores the vertices of the array.
	 */
	// Returns a handle to the model's vertices
	inline Vertex3Array& GetVertices(void)
	{
		return m_pVertices;
	}

	/** \brief Returns the array of vertices.
	 *
	 * \return Returns a const Vertex3Array reference that stores the vertices of the array.
	 */
	inline const Vertex3Array& GetVertices(void) const
	{
		return m_pVertices;
	}

	/** \brief Returns the array of vertex normals.
	 *
	 * \return Returns a reference to the array of vertex normals.
	 */
	// Returns a handle to the model's normals
	inline Normal3Array& GetNormals(void)
	{
		return m_pVertexNormals;
	}

	/** \brief Returns the array of texture coordinates.
	 *
	 * \return Returns a reference to the array of texture coordinates.
	 */
	// Returns a handle to the model's texture coordinates
	inline TCoordArray& GetTCoords(void)
	{
		return m_pTCoords;
	}

	/** \brief Returns the array of triangles of the surface triangulation.
	 *
	 * \return Returns a reference to the array of triangles of the model.
	 */
	// Returns the models faces
	inline TriFaceArray& GetFaces(void)
	{
		return m_pFaces;
	}

	/** \brief Returns false if there are problems in the model.
	 *
	 */
	//Checks for a valid model
	inline bool IsValid() {return m_bValid;};

	/** \brief Returns true if the model has a texture attached.
	 *
	 */
	inline bool IsTextured() {return m_bIsTextured;};

	/** \brief Returns the number of vertices in the surface triangulation.
	 *
	 * \return Returns the number of vertices in the mesh.
	 */
	//returns the number of vertices
	inline int NumVertices(){return m_iNumVerts;};

	/** \brief Returns the number of texture coordinates.
	 *
	 * \return Returns the number of texture coordinates in the mesh.
	 */
	//returns the number of vertices
	inline int NumTCoords(){return m_iNumTCoords;};

	/** \brief Returns the number of triangles in the mesh.
	 *
	 * \return Returns in the number of triangles.
	 */
	//returns the number of faces
	inline int NumFaces(){return m_iNumFaces;};

	/** \brief Returns the material id of the model.
	 *
	 * \return Returns the id of the material attached to the model.
	 */
	inline int GetMaterialID(){return m_iID;};

	/** \brief Sets the number of vertices in the model.
	 *
	 * \param iVerts Number of vertices determined during creation.
	 */
	inline void SetNumVertices(int iVerts){m_iNumVerts=iVerts;};
	
	/** \brief Sets the number of texture coordinates in the model.
	 *
	 * \param iTCoords Number of texture coordinates determined during creation.
	 */
	inline void SetNumTCoords(int iTCoords){m_iNumTCoords=iTCoords;};

	/** \brief Sets the number of triangles in the model.
	 *
	 * \param iFaces Number of triangles determined during creation.
	 */	inline void SetNumFaces(int iFaces){m_iNumFaces=iFaces;};

	/** \brief Set a flag if the model has a texture attached.
	 *
	 * \param bTexture true if the model has a texture.
	 */
	inline void SetTextured(bool bTexture) {m_bIsTextured=bTexture;};

	/** \brief Set the material if of the model.
	 *
	 * \param iID ID of the material attached to the model.
	 */
	inline void SetMaterialID(int iID) {m_iID=iID;};

	/** \brief Initialized the model by loading from a file.
	 *
	 * \param strFileName Name of the file to load.
	 */
	//Loads a model from file
	void LoadModel(char *strFileName);
	
	/** \brief Generates an AABB.
	 *
	 *  GenerateBoundingBox() Generates an axis oriented bounding box.
	 */
	void GenerateBoundingBox();	
	
	/** \brief Returns the bounding box of the mesh
	 *
	 * \return Returns the bounding box m_bdBox of the mesh
	 */
	inline const AABB3r& GetBox() {return m_bdBox;}
	inline const AABB3r& GetBox() const {return m_bdBox;}	
	
	inline void OutBox()
	{
	std::cout<<m_bdBox.vertices_[0].x<<" "<<m_bdBox.vertices_[0].y<<" "<<m_bdBox.vertices_[0].z<<std::endl;
	std::cout<<m_bdBox.vertices_[1].x<<" "<<m_bdBox.vertices_[1].y<<" "<<m_bdBox.vertices_[1].z<<std::endl;
	}
	VECTOR3 TransformModelWorldSingle(const VECTOR3 &vVec);

	void TransformModelWorld();

	VECTOR3 TransfromWorldModelSingle(const VECTOR3 &vVec);

	void MoveToPosition(const VECTOR3 &vPos)
	{
		m_vOrigin = vPos;
	}
	
	void CreateFrom(std::vector<VECTOR3> &vVertices,  std::vector<TriFace> &vFaces);

	FaceIter begin();
	FaceIter end();

	ConstFaceIter begin() const;
	ConstFaceIter end() const;
	
	
	MeshVertexIter MeshVertexBegin();
	MeshVertexIter MeshVertexEnd();
	
	void BuildVertexArrays(void);
	
	/** \brief Calculated vertex normals for the mesh from face normals.
	 *
	 */
	void CalcVertexNormals();

//<member_variables>
	/** \brief False if there are problems in the mesh.
	*
	*/
	bool                  m_bValid;

	/** \brief True if the model has a texture.
	*
	*/
	bool                  m_bIsTextured;
	//the name of the mesh object

	/** \brief Holds the name of the mesh.
	*
	*/
	std::string           m_strName;

	/** \brief Number of vertices.
	*
	*/
	int					  m_iNumVerts;

	/** \brief Number of texture coordinates.
	*
	*/
	int					  m_iNumTCoords;

	/** \brief Number of faces (triangles).
	*
	*/
	int					  m_iNumFaces;

	/** \brief Material id of the mesh.
	*
	*/
	int					  m_iID;

	/** \brief The center of mass of the model.
	*
	*/
	VECTOR3 m_vOrigin;
	
	/** \brief Rotation of the model.
	*
	* Rotation matrix
  *
	*/
  MATRIX3X3 m_matTransform;

	/** \brief The array of vertices.
	*
	*/
	Vertex3Array          m_pVertices;

	/** \brief The array of texture coordinates.
	*
	*/
	TCoordArray           m_pTCoords;

	/** \brief The array of vertex normals.
	*
	*/
	Vertex3Array m_pVertexNormals;

	/** \brief The array of faces.
	*
	*/
	TriFaceArray          m_pFaces;

	/** \brief The array of indices.
	*
	*/
	unsigned int*         m_pIndices;
	
	/** \brief Bounding box of the model.
	*
	*/
	AABB3r               m_bdBox;

  /** 
  *  Array of triangle bounding boxes
  */
  AABB3r               *triangleAABBs_;

  /**
  *  Generate bounding boxes for all triangles
  */
  void generateTriangleBoundingBoxes();



inline bool GetValid() {return m_bValid;};

inline bool GetIsTextured() {return m_bIsTextured;};

inline std::string GetName() {return m_strName;};

inline int GetNumVerts() {return m_iNumVerts;};

inline int GetNumTCoords() {return m_iNumTCoords;};

inline int GetNumFaces() {return m_iNumFaces;};

inline int GetID() {return m_iID;};

inline VECTOR3 GetOrigin() {return m_vOrigin;};

inline MATRIX3X3 GetTransformation() {return m_matTransform;};

inline Vertex3Array GetVertexNormals() {return m_pVertexNormals;};

inline unsigned int* GetIndices() {return m_pIndices;};

inline void SetValid(bool Valid) {m_bValid=Valid;};

inline void SetIsTextured(bool IsTextured) {m_bIsTextured=IsTextured;};

inline void SetName(const char Name[]) { m_strName=std::string(Name);};

inline void SetNumVerts(int NumVerts) {m_iNumVerts=NumVerts;};

inline void SetID(int ID) {m_iID=ID;};

inline void SetOrigin(VECTOR3 Origin) {m_vOrigin=Origin;};

inline void SetTransformation(const MATRIX3X3 &transformation) {m_matTransform=transformation;};

inline void SetVertices(Vertex3Array Vertices) {m_pVertices=Vertices;};

inline void SetTCoords(TCoordArray TCoords) {m_pTCoords=TCoords;};

inline void SetVertexNormals(Vertex3Array VertexNormals) {m_pVertexNormals=VertexNormals;};

inline void SetFaces(TriFaceArray Faces) {m_pFaces=Faces;};

inline void SetIndices(unsigned int* Indices) {m_pIndices=Indices;};

inline void SetBox(AABB3r Box) {m_bdBox=Box;};


};

}

#endif
