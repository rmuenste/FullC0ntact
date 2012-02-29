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
#ifndef INTERSECTORCIRCLECIRCLE_H
#define INTERSECTORCIRCLECIRCLE_H


#include <sphere.h>
#include <vector2.h>

namespace i3d {

/**
* @brief Computes whether two spheres intersect
*
* Computes whether two spheres intersect
*
*/        
template<class T>
class CIntersectorCircleCircle
{
  public:  
  CIntersectorCircleCircle(const CVector2<T> c0, const CVector2<T> c1, T r0, T r1)
  {
	  m_pC0 = c0;
	  m_pC1 = c1;
    m_r0  = r0;
    m_r1  = r1;
  };

  bool Find();
  CVector2<T> m_pC0;
  CVector2<T> m_pC1;
  T m_r0;
  T m_r1;

  CVector2<T> m_vPoints[4];

  int m_iIntersectionType;
  int m_iNumIntersections;

  enum
  {
    NONE,
    TOUCHING,
    INTERSECTING,
    EQUAL
  };

};

}

#endif // INTERSECTORCIRCLECIRCLE_H
