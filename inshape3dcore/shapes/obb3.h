/*
    <one line to give the program's name and a brief idea of what it does.>
    Copyright (C) <year>  <name of author>

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License along
    with this program; if not, write to the Free Software Foundation, Inc.,
    51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.

*/

#ifndef OBB3_H
#define OBB3_H

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
#include <segment3.h>
#include <shape.h>
#include <convexshape.h>
#include <rectangle3.h>

namespace i3d {


/** @brief An oriented bounding box in 3d space
 *
 * An oriented bounding box in 3d space, this class is also used
 * to represent a box in 3d without actually bounding something
 */ 
template <typename T>
class COBB3 : public CConvexShape<T>
{
public:

    COBB3();
    ~COBB3();

    COBB3(const CVector3<T>& center, const CVector3<T> axis[3], const T extent[3]);

    COBB3(const CVector3<T>& center, const CVector3<T>& axis0,
          const CVector3<T>& axis1, const CVector3<T>& axis2,
          const T extent0, const T extent1, const T extent2);

    COBB3(const COBB3<T> &copy);

    void ComputeVertices (CVector3<T> vertex[8]) const;

    bool PointInside(const CVector3<T> &vQuery) const;    

    T    GetMaximumExtent() const;

    T GetBoundingSphereRadius() const;

    unsigned int ClassifyVertexOnSurface(const CVector3<T> &pVertex) const;

    unsigned int ClassifyVertex(const CVector3<T> &pVertex) const;

    CVector3<T> GetSupport(const CVector3<T> &v) const
    {
      T sign[3];
      sign[0] = (v*m_vUVW[0] > 0) ? T(1.0) : T(-1.0);
      sign[1] = (v*m_vUVW[1] > 0) ? T(1.0) : T(-1.0);
      sign[2] = (v*m_vUVW[2] > 0) ? T(1.0) : T(-1.0);

      return (m_vCenter + sign[0]*m_vUVW[0]*m_Extents[0] + sign[1]*m_vUVW[1]*m_Extents[1] + sign[2]*m_vUVW[2]*m_Extents[2]);
    };

    CVector3<T> GetPointOnBoundary() const
    {
      //return top point
      return m_vCenter + m_Extents[0] * m_vUVW[0] + m_Extents[1] * m_vUVW[1] + m_Extents[2] * m_vUVW[2];
    };

    inline int GetRegionType(unsigned int regionCode) const
    {
      int count = 0;
      while(regionCode)
      {
        //AND with 0x01 and add up
        count += regionCode & 0x1u;
        //shift the bit away
        regionCode >>= 1;
      }
      return (4-count);
    }


/**
 *
 * Returns the vertex with the given index
 * @param index The vertex index
 * @return The vertex with the given index
 */
    CVector3<T> GetVertex(int index) const;

/**
 *
 * Returns an aabb for the obb
 * @return The axis-aligned bounding box of the obb
 */
	CAABB3<T> GetAABB();

/** 
 *
 * Calculates the volume of the box
  * \return The volume of the box
 */
	inline T Volume() const
	{
		
		T volume= 8.0 * m_Extents[0] * m_Extents[1] * m_Extents[2];

		return volume;
	}

  /**
   * Returns the geometric center of the shape
   *
   */
  CVector3<T> GetCenter() const {return m_vCenter;};

  CVector3<T> GetRegionVertex(unsigned int iRegion) const;
  CSegment3<T> GetRegionEdge(unsigned int iRegion) const;
  CRectangle3<T> GetRegionFace(unsigned int iRegion) const;
  CVector3<T> GetFaceNormal(unsigned int iRegion) const;
  void GetFacesAtEdge(unsigned int iRegion, unsigned int faces[2]) const;

    CVector3<T> m_vCenter;
    CVector3<T> m_vUVW[3];
    T m_Extents[3];
};

  enum
  {
    VERTEX=1,
    EDGE,
    FACE,
    INNER
  };

typedef COBB3<float> COBB3f;
typedef COBB3<double> COBB3d;
typedef COBB3<Real> COBB3r;

}

#endif // OBB3_H
