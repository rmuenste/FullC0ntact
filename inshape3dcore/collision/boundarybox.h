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

#ifndef BOUNDARYBOX_H
#define BOUNDARYBOX_H


#include <aabb3.h>
#include <shape.h>
#include <convexshape.h>
#include <rectangle3.h>

namespace i3d {

/**
* @brief A box-shaped boundary
* 
* A box-shaped boundary for the simulation
*/
template <class T>
class CBoundaryBox : public ConvexShape<T>
{
public:

	CBoundaryBox(void);
	CBoundaryBox(const CVector3<T> &vOrigin, const T extends[3]);

	~CBoundaryBox(void);

	CAABB3<T> getAABB() {return rBox;};

	T getVolume() const {return T(rBox.Volume());};

  CVector3<T> getSupport(const CVector3<T> &v) const {return CVector3<T>(0,0,0);};

  CVector3<T> getPointOnBoundary() const {return CVector3<T>(0,0,0);};

  void translateTo(const CVector3<T> &vPos)
  {

  };

  /**
   * Returns the geometric center of the shape
   *
   */
  CVector3<T> getCenter() const {return rBox.GetCenter();};

  bool isPointInside(const CVector3<T> &vQuery) const
  {
    //TODO:implement 
    return false; 
  }  

  void setBoundaryType(int itype)
  {
    m_iType = itype;
  };

  int getBoundaryType()
  {
    return m_iType;
  };


	CAABB3<T> rBox;
	void CalcValues();

	T m_Values[6];

	CVector3<T> m_vNormals[6];
	CVector3<T> m_vPoints[6];

  std::vector< CRectangle3<T> > m_vBorders;

  int m_iType;

	enum
	{
		BOXBDRY,
    CYLBDRY
	};
  
};

/* typedefs to create float and double vectors */
typedef CBoundaryBox<double> CBoundaryBoxd;
typedef CBoundaryBox<float>  CBoundaryBoxf;
typedef CBoundaryBox<Real>  CBoundaryBoxr;

}

#endif
