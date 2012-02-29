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
#ifndef INTERSECTORSPHERESEGMENT_H
#define INTERSECTORSPHERESEGMENT_H

#include <sphere.h>
#include <segment3.h>

namespace i3d {

/**
* @brief Computes whether two spheres intersect
*
* Computes whether a spheres and a line intersect
*
*/
template<class T>
class CIntersectorSphereSegment
{
  public:
  CIntersectorSphereSegment(const CSphere<T> &sphere, const CSegment3<T> &segment)
  {
	  m_pSphere = &sphere;
	  m_pSegment = &segment;
  };

  bool Intersection();

  const CSphere<T>   *m_pSphere;
  const CSegment3<T> *m_pSegment;

  int m_iNumIntersections;
  int m_iIntersectionType;

  T   m_dParameterSegment[2];
  CVector3<T>   m_vPoints[2];

  enum
  {
    EMPTY,
    SEGMENT,
    POINT
  };
  
  
};

typedef CIntersectorSphereSegment<Real> CIntersectorSphereSegmentr;

}

#endif // INTERSECTORSPHERESEGMENT_H
