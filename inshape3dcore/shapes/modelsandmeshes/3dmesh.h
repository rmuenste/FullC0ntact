
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


/** \brief A Mesh3D is a class that holds actual mesh data like vertices and connectivity.
 *
 * A Mesh3D is a class that holds actual mesh data like vertices and connectivity. A C3DModel can be composed of one or more
 * Mesh3Des.
 */
class Mesh3D
{
public:

	/** \brief Standard constructor for the Mesh3D class
	 *
	 */
	Mesh3D(void);

	/** 
	 * Copies a Mesh3D to another
	 */	
	Mesh3D(const Mesh3D &pMesh);

	/** 
	 *
	 * A more extensive description of myProcedure().
	 * \param aParameter A brief description of aParameter.
	 */
	Mesh3D(char *strName);

	/** \brief Destructor.
	 *
	 */
	~Mesh3D(void);

	/** \brief Returns the array of vertices.
	 *
	 * \return Returns a Vertex3Array reference that stores the vertices of the array.
	 */
	// Returns a handle to the model's vertices
	inline Vertex3Array& getVertices(void)
	{
		return vertices_;
	}

	/** \brief Returns the array of vertices.
	 *
	 * \return Returns a const Vertex3Array reference that stores the vertices of the array.
	 */
	inline const Vertex3Array& GetVertices(void) const
	{
		return vertices_;
	}

	/** \brief Returns the array of vertex normals.
	 *
	 * \return Returns a reference to the array of vertex normals.
	 */
	// Returns a handle to the model's normals
	inline Normal3Array& getNormals(void)
	{
		return vertexNormals_;
	}

	/** \brief Returns the array of texture coordinates.
	 *
	 * \return Returns a reference to the array of texture coordinates.
	 */
	// Returns a handle to the model's texture coordinates
	inline TCoordArray& getTexCoords(void)
	{
		return texCoords_;
	}

	/** \brief Returns the array of triangles of the surface triangulation.
	 *
	 * \return Returns a reference to the array of triangles of the model.
	 */
	// Returns the models faces
	inline TriFaceArray& getFaces(void)
	{
		return faces_;
	}

	/** \brief Returns false if there are problems in the model.
	 *
	 */
	//Checks for a valid model
	inline bool isValid() {return valid_;};

	/** \brief Returns true if the model has a texture attached.
	 *
	 */
	inline bool isTextured() {return textured_;};

	/** \brief Returns the number of vertices in the surface triangulation.
	 *
	 * \return Returns the number of vertices in the mesh.
	 */
	//returns the number of vertices
	inline int getNumVertices(){return numVerts_;};

	/** \brief Returns the number of texture coordinates.
	 *
	 * \return Returns the number of texture coordinates in the mesh.
	 */
	//returns the number of vertices
	inline int getNumTexCoords(){return numTexCoords_;};

	/** \brief Returns the number of triangles in the mesh.
	 *
	 * \return Returns in the number of triangles.
	 */
	//returns the number of faces
	inline int getNumFaces(){return numFaces_;};

	/** \brief Returns the material id of the model.
	 *
	 * \return Returns the id of the material attached to the model.
	 */
	inline int getMatId(){return matId_;};

	/** \brief Sets the number of vertices in the model.
	 *
	 * \param iVerts Number of vertices determined during creation.
	 */
	inline void setNumVertices(int iVerts){numVerts_=iVerts;};
	
	/** \brief Sets the number of texture coordinates in the model.
	 *
	 * \param iTCoords Number of texture coordinates determined during creation.
	 */
	inline void setNumTexCoords(int iTCoords){numTexCoords_=iTCoords;};

	/** \brief Sets the number of triangles in the model.
	 *
	 * \param iFaces Number of triangles determined during creation.
	 */	inline void setNumFaces(int iFaces){numFaces_=iFaces;};

	/** \brief Set a flag if the model has a texture attached.
	 *
	 * \param bTexture true if the model has a texture.
	 */
	inline void setTextured(bool bTexture) {textured_=bTexture;};

	/** \brief Initialized the model by loading from a file.
	 *
	 * \param strFileName Name of the file to load.
	 */
	//Loads a model from file
	void loadModel(char *strFileName);
	
	/** \brief Generates an AABB.
	 *
	 *  GenerateBoundingBox() Generates an axis oriented bounding box.
	 */
	void generateBoundingBox();
	
	/** \brief Returns the bounding box of the mesh
	 *
	 * \return Returns the bounding box m_bdBox of the mesh
	 */
	inline const AABB3r& getBox() const {return box_;}

	inline void outBox()
	{
	std::cout<<box_.vertices_[0].x<<" "<<box_.vertices_[0].y<<" "<<box_.vertices_[0].z<<std::endl;
	std::cout<<box_.vertices_[1].x<<" "<<box_.vertices_[1].y<<" "<<box_.vertices_[1].z<<std::endl;
	}
	VECTOR3 TransformModelWorldSingle(const VECTOR3 &vVec);

	void TransformModelWorld();

	VECTOR3 TransfromWorldModelSingle(const VECTOR3 &vVec);

	void moveToPosition(const VECTOR3 &vPos)
	{
		com_ = vPos;
	}
	
	void createFrom(std::vector<VECTOR3> &vVertices,  std::vector<TriFace> &vFaces);

	FaceIter begin();
	FaceIter end();

	ConstFaceIter begin() const;
	ConstFaceIter end() const;
	
	
	MeshVertexIter meshVertexBegin();
	MeshVertexIter meshVertexEnd();
	
	void buildVertexArrays(void);
	
	/** \brief Calculated vertex normals for the mesh from face normals.
	 *
	 */
	void calcVertexNormals();

//<member_variables>
	/** \brief False if there are problems in the mesh.
	*
	*/
	bool                  valid_;

	/** \brief True if the model has a texture.
	*
	*/
	bool                  textured_;
	//the name of the mesh object

	/** \brief Holds the name of the mesh.
	*
	*/
	std::string           fileName_;

	/** \brief Number of vertices.
	*
	*/
	int					  numVerts_;

	/** \brief Number of texture coordinates.
	*
	*/
	int					  numTexCoords_;

	/** \brief Number of faces (triangles).
	*
	*/
	int					  numFaces_;

	/** \brief Material id of the mesh.
	*
	*/
	int					  matId_;

	/** \brief The center of mass of the model.
	*
	*/
	VECTOR3 com_;
	
	/** \brief Rotation of the model.
	*
	* Rotation matrix
  *
	*/
  MATRIX3X3 transform_;

	/** \brief The array of vertices.
	*
	*/
	Vertex3Array          vertices_;

	/** \brief The array of texture coordinates.
	*
	*/
	TCoordArray           texCoords_;

	/** \brief The array of vertex normals.
	*
	*/
	Vertex3Array vertexNormals_;

	/** \brief The array of faces.
	*
	*/
	TriFaceArray          faces_;

	/** \brief The array of indices.
	*
	*/
	unsigned int*         indices_;
	
	/** \brief Bounding box of the model.
	*
	*/
	AABB3r               box_;

  /** 
  *  Array of triangle bounding boxes
  */
  AABB3r               *triangleAABBs_;

  /**
  *  Generate bounding boxes for all triangles
  */
  void generateTriangleBoundingBoxes();



inline bool getValid() {return valid_;};

inline bool getIsTextured() {return textured_;};

inline std::string getFileName() {return fileName_;};

inline int getNumVerts() {return numVerts_;};

inline int getMaterialId() {return matId_;};

inline VECTOR3 getCOM() {return com_;};

inline MATRIX3X3 getTransform() {return transform_;};

inline Vertex3Array getVertexNormals() {return vertexNormals_;};

inline unsigned int* getIndices() {return indices_;};

inline void setValid(bool Valid) {valid_=Valid;};

inline void setIsTextured(bool IsTextured) {textured_=IsTextured;};

inline void setFileName(const char Name[]) { fileName_=std::string(Name);};

inline void setNumVerts(int NumVerts) {numVerts_=NumVerts;};

/** \brief Set the material if of the model.
 *
 * \param iID ID of the material attached to the model.
 */

inline void setMaterialId(int ID) {matId_=ID;};

inline void setCOM(VECTOR3 Origin) {com_=Origin;};

inline void setTransform(const MATRIX3X3 &transformation) {transform_=transformation;};

inline void detVertices(Vertex3Array Vertices) {vertices_=Vertices;};

inline void setTexCoords(TCoordArray TCoords) {texCoords_=TCoords;};

inline void setVertexNormals(Vertex3Array VertexNormals) {vertexNormals_=VertexNormals;};

inline void setFaces(TriFaceArray Faces) {faces_=Faces;};

inline void setIndices(unsigned int* Indices) {indices_=Indices;};

inline void setBox(AABB3r Box) {box_=Box;};


};

}

#endif
