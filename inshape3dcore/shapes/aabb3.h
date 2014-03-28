/*
   This library is free software; you can redistribute it and/or
   modify it under the terms of the GNU Library General Public
   License version 2 as published by the Free Software Foundation.

   This library is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
   Library General Public License for more details.

   You should have received a copy of the GNU Library General Public License
   along with this library; see the file COPYING.LIB.  If not, write to
   the Free Software Foundation, Inc., 51 Franklin Street, Fifth Floor,
   Boston, MA 02110-1301, USA.
*/

#ifndef _AABB3_H
#define _AABB3_H

//===================================================
//					DEFINITIONS
//===================================================


//===================================================
//					INCLUDES
//===================================================
#include <iostream>
#include <vector>
#include <limits>
#include <vector3.h>
#include <dynamicarray.h>
#include "triangle3.h"

namespace i3d {

/** \brief A class for an axis-aligned bounding box
 *
 * Class with functions for an axis aligned bounding box
 */
template<class T>
class CAABB3
{
public:

/** 
 * Standard constructor
 */
	CAABB3(void){};

/** 
 *
 * Copy constructor
 * 
 */
  CAABB3(const CAABB3<T> &copy)
  {
    center_ = copy.center_;
    m_Extends[0] = copy.m_Extends[0];
    m_Extends[1] = copy.m_Extends[1];
    m_Extends[2] = copy.m_Extends[2];
    vertices_[0]   = copy.vertices_[0];
    vertices_[1]   = copy.vertices_[1];
  };

/** \brief Constructs a CAABB3 from a sphere
 *
 * Constructs a CAABB3 from a sphere
 * \param vCenter Center of the box
 * \param rad extend of the three axes of the box
 */
	CAABB3(const CVector3<T> vCenter, T rad)
	{
		center_ = vCenter;
		m_Extends[0] = rad;
		m_Extends[1] = rad;
		m_Extends[2] = rad;
    vertices_[0]=center_-CVector3<T>(m_Extends[0],m_Extends[1],m_Extends[2]);
    vertices_[1]=center_+CVector3<T>(m_Extends[0],m_Extends[1],m_Extends[2]);
	};

/** \brief Constructs a CAABB3 from a center and extends
 *
 * Constructs a CAABB3 from a center and extends
 * \param vCenter Center of the box
 * \param extends array of the extents of the three axes
 */
	CAABB3(const CVector3<T> vCenter, T extends[])
	{
		center_ = vCenter;
		m_Extends[0] = extends[0];
		m_Extends[1] = extends[1];
		m_Extends[2] = extends[2];
		vertices_[0]   = CVector3<T>(vCenter.x-extends[0],vCenter.y-extends[1],vCenter.z-extends[2]);
		vertices_[1]   = CVector3<T>(vCenter.x+extends[0],vCenter.y+extends[1],vCenter.z+extends[2]);
	};

	

/** \brief Constructs an AABB from a point cloud
 *
 * Constructs an AABB from a point cloud, such that the box
 * contains all points in the cloud
 * \param Vec3Array The point cloud
 */
  CAABB3(const CDynamicArray< CVector3<T> > &Vec3Array);

/** \brief Constructs an AABB from two points
 *
 * Constructs an AABB from two points
 * \param vBL Left bottom point of the box
 * \param vTR right top point of the box
 */
	CAABB3(const CVector3<T> &vBL, const CVector3<T> &vTR);

/**
 * Destructor
 */
	~CAABB3(void){};

/**
 * Initialize an aabb from a point cloud
 */
	void InitBox(const CDynamicArray< CVector3<T> > &Vec3Array);

/**
 * Reset the vertices of the aabb
 */
	void SetBox(CVector3<T> minVec, CVector3<T> maxVec);

/**
 * Generate an aabb for a vector of triangles
 */
	void Init(const std::vector<CTriangle3<T> > &vTriangles);

/**
 * Generate an aabb for two extreme vertices
 */
	void Init(const CVector3<T> &minVec, const CVector3<T> &maxVec);

/**
 * Generate an aabb from min/max values
 */
	void Init(T minX,T minY,T minZ,T maxX,T maxY,T maxZ);
	
/**
 * Generate a box from a center vertex with certain (x,y,z)-extends
 */
	void Init(const CVector3<T> vCenter,const T extends[])
	{
		center_ = vCenter;
		m_Extends[0] = extends[0];
		m_Extends[1] = extends[1];
		m_Extends[2] = extends[2];
		vertices_[0]   = CVector3<T>(vCenter.x-extends[0],vCenter.y-extends[1],vCenter.z-extends[2]);
		vertices_[1]   = CVector3<T>(vCenter.x+extends[0],vCenter.y+extends[1],vCenter.z+extends[2]);
	}

/** 
 * Returns whether vQuery is inside the aabb
 */
	bool Inside(const CVector3<T> &vQuery) const;

/** 
 * Returns an integer (0,1,2) identifying either the (x,y,z) axes
 * as the longest
 */
	int LongestAxis() const;

/** \brief A brief description of MinDistanceDebug().
 *
 * A more extensive description of MinDistanceDebug().
 * \param aParameter A brief description of aParameter.
 * \return A brief description of what MinDistanceDebug() returns.
 */
	CVector3<T> MinDistanceDebug(const CVector3<T> &vQuery);

/** 
 * Returns the minimum distance of vQuery to the aabb
 */
	T MinDistance(const CVector3<T> &vQuery);

/** 
 * Returns the minimum squared distance of vQuery to the aabb
 */
	T MinDistanceSqr(const CVector3<T> &vQuery);

/** 
 * Returns the maximum distance of vQuery to the aabb
 */
  inline T MaxDistance(const CVector3<T> &vQuery) {return (CVector3<T>::createVector(vQuery,m_vUpper)).mag();};

/** 
 * Returns the maximum squared distance of vQuery to the aabb
 */
	inline T MaxDistanceSqr(const CVector3<T> &vQuery) {return (CVector3<T>::createVector(vQuery,m_vUpper)).norm2();};

/** \brief A brief description of update().
 *
 * A more extensive description of update().
 * \param aParameter A brief description of aParameter.
 * \return A brief description of what update() returns.
 */
	void update(const CVector3<T> &vQuery);

/** 
 * Return the front-bottom-left vertex
 */
	inline CVector3<T> GetFBL() const {return vertices_[0];};

/** 
 * Return the back-top-right vertex
 */
	inline CVector3<T> GetBTR() const {return vertices_[1];};

/** 
 * Return the front-bottom-right vertex
 */
	inline CVector3<T> GetFBR() const {return CVector3<T>(vertices_[1].x, vertices_[0].y, vertices_[0].z);};

/** 
 * Return the front-top-right vertex
 */
	inline CVector3<T> GetFTR() const {return CVector3<T>(vertices_[1].x, vertices_[1].y, vertices_[0].z);};

/** 
 * Return the front-top-left vertex
 */
	inline CVector3<T> GetFTL() const {return CVector3<T>(vertices_[0].x, vertices_[1].y, vertices_[0].z);};

/** 
 * Return the back-bottom-left vertex
 */
	inline CVector3<T> GetBBL() const {return CVector3<T>(vertices_[0].x, vertices_[0].y, vertices_[1].z);};

/** 
 * Return the back-bottom-right vertex
 */
	inline CVector3<T> GetBBR() const {return CVector3<T>(vertices_[1].x, vertices_[0].y, vertices_[1].z);};

/** 
 * Return the back-top-left vertex
 */
	inline CVector3<T> GetBTL() const {return CVector3<T>(vertices_[0].x, vertices_[1].y, vertices_[1].z);};

/**
 * Return the minimum x-coordinate
 */
	inline T Xmin() const {return vertices_[0].x;};

/**
 * Return the maximum x-coordinate
 */
	inline T Xmax() const {return vertices_[1].x;};

/**
 * Return the minimum y-coordinate
 */
	inline T Ymin() const {return vertices_[0].y;};

/**
 * Return the maximum y-coordinate
 */
	inline T Ymax() const {return vertices_[1].y;};

/**
 * Return the minimum z-coordinate
 */
	inline T Zmin() const {return vertices_[0].z;};

/**
 * Return the maximum z-coordinate
 */
	inline T Zmax() const {return vertices_[1].z;};

/** 
 * Return the center of the aabb
 */
	inline CVector3<T> GetCenter() const
	{
		return center_;
	};
	
	CVector3<T> GetVertex(int i);
	
/** \brief The function calculates and returns the volume of the box
 *
 * The function calculates and returns the volume of the box
 * \return The volume of the box
 */
	inline T Volume() const
	{
		
		T volume=(vertices_[1].x - vertices_[0].x) * (vertices_[1].y - vertices_[0].y) * (vertices_[1].z - vertices_[0].z);

		return volume;
	}
	
/** \brief A brief description of Output().
 *
 * The function calculates and returns the volume of the box.
 * \param aParameter No parameter required
 * \return The volume of the box
 */
	inline void Output() const
	{
		
		std::cout<<vertices_[0];
		std::cout<<vertices_[1];
		std::cout<<center_;
    std::cout<<"x dimension: "<<m_Extends[0]<<", y dimension: "<<m_Extends[1]<<", z dimension: "<<m_Extends[2]<<"\n";
	}

/** \brief Return the radius of a bounding sphere for the aabb
 *
 * Return the radius of a bounding sphere for the aabb
 * \return Radius of the bounding sphere
 */
  inline T GetBoundingSphereRadius() const
  {
    CVector3<T> vDiag = vertices_[1] - center_;
    return vDiag.mag();
  }

	enum
	{
		XAXIS,
		YAXIS,
		ZAXIS
	};

/**
  * Array of the bottom left and upper right vertices of the aabb
  */ 
	CVector3<T> vertices_[2];

  
/**
 * Array of the extends of the box on the xyz-axes relative to the
 * center of the aabb
 */
 T m_Extends[3];

/**
 * Center of the aabb
 */
 CVector3<T> center_;

/**
 * Storage for 3 values
 */
  CVector3<T> m_vUpper;

};

typedef CAABB3<float> CAABB3f;
typedef CAABB3<double> CAABB3d;
typedef CAABB3<Real> CAABB3r;

}
#endif

