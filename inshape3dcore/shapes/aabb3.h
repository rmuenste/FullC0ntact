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

/** \brief A brief description of CAABB3().
 *
 * A more extensive description of CAABB3().
 * \param aParameter A brief description of aParameter.
 * \return A brief description of what CAABB3() returns.
 */
	CAABB3(void){};

/** \brief A brief description of CAABB3(const CAABB3<T> &copy).
 *
 * Copy constructur
 * \copy the box to be copied
 */
	CAABB3(const CAABB3<T> &copy)
	{
		m_vCenter = copy.m_vCenter;
		m_Extends[0] = copy.m_Extends[0];
		m_Extends[1] = copy.m_Extends[1];
		m_Extends[2] = copy.m_Extends[2];
    m_Verts[0]   = copy.m_Verts[0];
    m_Verts[1]   = copy.m_Verts[1];
	};

/** \brief Constructs a CAABB3 from a sphere
 *
 * Constructs a CAABB3 from a sphere
 * \param vCenter Center of the box
 * \param rad extend of the three axes of the box
 */
	CAABB3(const CVector3<T> vCenter, T rad)
	{
		m_vCenter = vCenter;
		m_Extends[0] = rad;
		m_Extends[1] = rad;
		m_Extends[2] = rad;
    m_Verts[0]=m_vCenter-CVector3<T>(m_Extends[0],m_Extends[1],m_Extends[2]);
    m_Verts[1]=m_vCenter+CVector3<T>(m_Extends[0],m_Extends[1],m_Extends[2]);
	};

/** \brief Constructs a CAABB3 from a center and extends
 *
 * Constructs a CAABB3 from a center and extends
 * \param vCenter Center of the box
 * \param extends array of the extents of the three axes
 */
	CAABB3(const CVector3<T> vCenter, T extends[])
	{
		m_vCenter = vCenter;
		m_Extends[0] = extends[0];
		m_Extends[1] = extends[1];
		m_Extends[2] = extends[2];
		m_Verts[0]   = CVector3<T>(vCenter.x-extends[0],vCenter.y-extends[1],vCenter.z-extends[2]);
		m_Verts[1]   = CVector3<T>(vCenter.x+extends[0],vCenter.y+extends[1],vCenter.z+extends[2]);
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

/** \brief A brief description of ~CAABB3().
 *
 * A more extensive description of ~CAABB3().
 * \param aParameter A brief description of aParameter.
 * \return A brief description of what ~CAABB3() returns.
 */
	~CAABB3(void){};

/** \brief A brief description of InitBox().
 *
 * A more extensive description of InitBox().
 * \param aParameter A brief description of aParameter.
 * \return A brief description of what InitBox() returns.
 */
	void InitBox(const CDynamicArray< CVector3<T> > &Vec3Array);

/** \brief A brief description of SetBox().
 *
 * A more extensive description of SetBox().
 * \param aParameter A brief description of aParameter.
 * \return A brief description of what SetBox() returns.
 */
	void SetBox(CVector3<T> minVec, CVector3<T> maxVec);

/** \brief A brief description of Init().
 *
 * A more extensive description of Init(const CVector3<T> &minVec, const CVector3<T> &maxVec).
 * \param aParameter A brief description of aParameter.
 * \return A brief description of what Init(const CVector3<T> &minVec, const CVector3<T> &maxVec) returns.
 */
	void Init(const std::vector<CTriangle3<T> > &vTriangles);

/** \brief A brief description of myProcedure().
 *
 * A more extensive description of Init().
 * \param aParameter A brief description of aParameter.
 * \return A brief description of what Init() returns.
 */
	void Init(const CVector3<T> &minVec, const CVector3<T> &maxVec);

/** \brief A brief description of Init().
 *
 * A more extensive description of Init().
 * \param aParameter A brief description of aParameter.
 * \return A brief description of what Init() returns.
 */
	void Init(T minX,T minY,T minZ,T maxX,T maxY,T maxZ);
	
/** \brief A brief description of Init().
 *
 * A more extensive description of Init().
 * \param aParameter A brief description of aParameter.
 * \return A brief description of what Init() returns.
 */
	void Init(const CVector3<T> vCenter,const T extends[])
	{
		m_vCenter = vCenter;
		m_Extends[0] = extends[0];
		m_Extends[1] = extends[1];
		m_Extends[2] = extends[2];
		m_Verts[0]   = CVector3<T>(vCenter.x-extends[0],vCenter.y-extends[1],vCenter.z-extends[2]);
		m_Verts[1]   = CVector3<T>(vCenter.x+extends[0],vCenter.y+extends[1],vCenter.z+extends[2]);
	}

/** \brief A brief description of Inside().
 *
 * A more extensive description of Inside().
 * \param aParameter A brief description of aParameter.
 * \return A brief description of what Inside() returns.
 */
	bool Inside(const CVector3<T> &vQuery) const;

/** \brief A brief description of LongestAxis().
 *
 * A more extensive description of LongestAxis().
 * \param aParameter A brief description of aParameter.
 * \return A brief description of what LongestAxis() returns.
 */
	int LongestAxis() const;

/** \brief A brief description of MinDistanceDebug().
 *
 * A more extensive description of MinDistanceDebug().
 * \param aParameter A brief description of aParameter.
 * \return A brief description of what MinDistanceDebug() returns.
 */
	CVector3<T> MinDistanceDebug(const CVector3<T> &vQuery);

/** \brief A brief description of MinDistance().
 *
 * A more extensive description of MinDistance().
 * \param aParameter A brief description of aParameter.
 * \return A brief description of what MinDistance() returns.
 */
	T MinDistance(const CVector3<T> &vQuery);

/** \brief A brief description of MinDistanceSqr().
 *
 * A more extensive description of MinDistanceSqr().
 * \param aParameter A brief description of aParameter.
 * \return A brief description of what MinDistanceSqr() returns.
 */
	T MinDistanceSqr(const CVector3<T> &vQuery);

/** \brief A brief description of MaxDistance().
 *
 * A more extensive description of MaxDistance().
 * \param aParameter A brief description of aParameter.
 * \return A brief description of what MaxDistance() returns.
 */
  inline T MaxDistance(const CVector3<T> &vQuery) {return (CVector3<T>::createVector(vQuery,m_vUpper)).mag();};

/** \brief A brief description of MaxDistanceSqr().
 *
 * A more extensive description of MaxDistanceSqr().
 * \param aParameter A brief description of aParameter.
 * \return A brief description of what MaxDistanceSqr() returns.
 */
	inline T MaxDistanceSqr(const CVector3<T> &vQuery) {return (CVector3<T>::createVector(vQuery,m_vUpper)).norm2();};

/** \brief A brief description of update().
 *
 * A more extensive description of update().
 * \param aParameter A brief description of aParameter.
 * \return A brief description of what update() returns.
 */
	void update(const CVector3<T> &vQuery);

	//inline methods to access vertices
/** \brief A brief description of GetFBL().
 *
 * A more extensive description of GetFBL().
 * \param aParameter A brief description of aParameter.
 * \return A brief description of what GetFBL() returns.
 */
	inline CVector3<T> GetFBL() const {return m_Verts[0];};

/** \brief A brief description of GetBTR().
 *
 * A more extensive description of GetBTR().
 * \param aParameter A brief description of aParameter.
 * \return A brief description of what GetBTR() returns.
 */
	inline CVector3<T> GetBTR() const {return m_Verts[1];};

/** \brief A brief description of GetFBR().
 *
 * A more extensive description of GetFBR().
 * \param aParameter A brief description of aParameter.
 * \return A brief description of what GetFBR() returns.
 */
	inline CVector3<T> GetFBR() const {return CVector3<T>(m_Verts[1].x, m_Verts[0].y, m_Verts[0].z);};

/** \brief A brief description of GetFTR().
 *
 * A more extensive description of GetFTR().
 * \param aParameter A brief description of aParameter.
 * \return A brief description of what GetFTR() returns.
 */
	inline CVector3<T> GetFTR() const {return CVector3<T>(m_Verts[1].x, m_Verts[1].y, m_Verts[0].z);};

/** \brief A brief description of GetFTL().
 *
 * A more extensive description of GetFTL().
 * \param aParameter A brief description of aParameter.
 * \return A brief description of what GetFTL() returns.
 */
	inline CVector3<T> GetFTL() const {return CVector3<T>(m_Verts[0].x, m_Verts[1].y, m_Verts[0].z);};

/** \brief A brief description of GetBBL().
 *
 * A more extensive description of GetBBL().
 * \param aParameter A brief description of aParameter.
 * \return A brief description of what GetBBL() returns.
 */
	inline CVector3<T> GetBBL() const {return CVector3<T>(m_Verts[0].x, m_Verts[0].y, m_Verts[1].z);};

/** \brief A brief description of GetBBR().
 *
 * A more extensive description of GetBBR().
 * \param aParameter A brief description of aParameter.
 * \return A brief description of what GetBBR() returns.
 */
	inline CVector3<T> GetBBR() const {return CVector3<T>(m_Verts[1].x, m_Verts[0].y, m_Verts[1].z);};

/** \brief A brief description of GetBTL().
 *
 * A more extensive description of GetBTL().
 * \param aParameter A brief description of aParameter.
 * \return A brief description of what GetBTL() returns.
 */
	inline CVector3<T> GetBTL() const {return CVector3<T>(m_Verts[0].x, m_Verts[1].y, m_Verts[1].z);};

/** \brief A brief description of Xmin().
 *
 * A more extensive description of Xmin().
 * \param aParameter A brief description of aParameter.
 * \return A brief description of what Xmin() returns.
 */
	inline T Xmin() const {return m_Verts[0].x;};

/** \brief A brief description of Xmax().
 *
 * A more extensive description of Xmax().
 * \param aParameter A brief description of aParameter.
 * \return A brief description of what Xmax() returns.
 */
	inline T Xmax() const {return m_Verts[1].x;};

/** \brief A brief description of Ymin().
 *
 * A more extensive description of Ymin().
 * \param aParameter A brief description of aParameter.
 * \return A brief description of what Ymin() returns.
 */
	inline T Ymin() const {return m_Verts[0].y;};

/** \brief A brief description of Ymax().
 *
 * A more extensive description of Ymax().
 * \param aParameter A brief description of aParameter.
 * \return A brief description of what Ymax() returns.
 */
	inline T Ymax() const {return m_Verts[1].y;};

/** \brief A brief description of Zmin().
 *
 * A more extensive description of Zmin().
 * \param aParameter A brief description of aParameter.
 * \return A brief description of what Zmin() returns.
 */
	inline T Zmin() const {return m_Verts[0].z;};

/** \brief A brief description of Zmax().
 *
 * A more extensive description of Zmax().
 * \param aParameter A brief description of aParameter.
 * \return A brief description of what Zmax() returns.
 */
	inline T Zmax() const {return m_Verts[1].z;};

/** \brief A brief description of GetCenter().
 *
 * A more extensive description of GetCenter().
 * \param aParameter A brief description of aParameter.
 * \return A brief description of what GetCenter() returns.
 */
	inline CVector3<T> GetCenter() const
	{
		return m_vCenter;
	};
	
	CVector3<T> GetVertex(int i);
	
/** \brief A brief description of Volume().
 *
 * The function calculates and returns the volume of the box.
 * \param aParameter No parameter required
 * \return The volume of the box
 */
	inline T Volume() const
	{
		
		T volume=(m_Verts[1].x - m_Verts[0].x) * (m_Verts[1].y - m_Verts[0].y) * (m_Verts[1].z - m_Verts[0].z);

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
		
		std::cout<<m_Verts[0];
		std::cout<<m_Verts[1];
		std::cout<<m_vCenter;

	}	
	
  inline T GetBoundingSphereRadius() const
  {
    CVector3<T> vDiag = m_Verts[1] - m_vCenter;
    return vDiag.mag();
  }

	enum
	{
		XAXIS,
		YAXIS,
		ZAXIS
	};

	CVector3<T> m_Verts[2];

	//store the extends of the box
	T m_Extends[3];

	CVector3<T> m_vCenter;

	//could make AABB with for branch & bound by inheritance
	CVector3<T> m_vUpper;

};

typedef CAABB3<float> CAABB3f;
typedef CAABB3<double> CAABB3d;
typedef CAABB3<Real> CAABB3r;

}
#endif

