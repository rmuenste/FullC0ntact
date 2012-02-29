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

#include "intersectorcirclecircle.h"

namespace i3d {

template<class T>
bool CIntersectorCircleCircle<T>::Find()
{
  //the algorithm is taken from Eberly
  //difference between the centers
  CVector2<T> u = m_pC1 - m_pC0;

  T uLenSqr     = u * u;

  T diffRad     = m_r1 - m_r0;

  if(uLenSqr < CMath<T>::TOLERANCEZERO && diffRad < CMath<T>::TOLERANCEZERO)
  {
    //the circles are the same
    CVector2<T> e0(1,0);
    CVector2<T> e1(0,1);
    m_vPoints[0]=m_pC0 + m_r0 * e0;
    m_vPoints[1]=m_pC0 + m_r0 * e1;
    m_vPoints[2]=m_pC0 - m_r0 * e0;
    m_vPoints[3]=m_pC0 - m_r0 * e1;
    m_iIntersectionType = EQUAL;
    m_iNumIntersections = 4;
    return true;
  }

  T diffRadSqr = diffRad * diffRad;
  if(uLenSqr < diffRadSqr)
  {
    CVector2<T> e0(1,0);
    CVector2<T> e1(0,1);
    if(m_r0 < m_r1)
    {
      m_vPoints[0]=m_pC0 + m_r0 * e0;
      m_vPoints[1]=m_pC0 + m_r0 * e1;
      m_vPoints[2]=m_pC0 - m_r0 * e0;
      m_vPoints[3]=m_pC0 - m_r0 * e1;
    }
    else
    {
      m_vPoints[0]=m_pC1 + m_r1 * e0;
      m_vPoints[1]=m_pC1 + m_r1 * e1;
      m_vPoints[2]=m_pC1 - m_r1 * e0;
      m_vPoints[3]=m_pC1 - m_r1 * e1;
    }
    m_iIntersectionType = NONE;
    m_iNumIntersections = 4;
    return true;
  }

  T sumRad    = m_r0 + m_r1;
  T sumRadSqr = sumRad * sumRad;
  //if the squared length of u is larger than |(r1+r0)|^2
  //there are no real roots
  if(uLenSqr > sumRadSqr)
  {
    m_iIntersectionType = NONE;
    m_iNumIntersections = 0;
    return false;
  }

  if(uLenSqr < sumRadSqr)
  {
    if(diffRadSqr < uLenSqr)
    {
      T invuSqr = T(1.0)/uLenSqr;
      T s = T(0.5) * (((m_r0*m_r0-m_r1*m_r1)/invuSqr) + T(1.0));

      CVector2<T> tmp = m_pC0 + s * u;
      T discr = m_r0*m_r0*invuSqr - s*s;
      if(discr < T(0))
      {
        discr = T(0);
      }
      T t = sqrt(discr);
      CVector2<T> v(u.y,-u.x);
      m_vPoints[0] = tmp - t*v;
      m_vPoints[1] = tmp + t*v;

      //normalize u
      u.Normalize();
      m_vPoints[2]=m_pC0 + m_r0 * u;
      m_vPoints[3]=m_pC1 - m_r1 * u;

      m_iIntersectionType = INTERSECTING;
      m_iNumIntersections = 4;
    }
    else
    {
      //|u| is not bigger |r0-r1| and not smaller |r0-r1|
      //circles are tangent
      m_vPoints[0] = m_pC0 + (m_r0/sumRad)*u;
      m_iNumIntersections = 1;
      m_iIntersectionType = TOUCHING;
    }
  }
  else
  {
    //|u| is not bigger |r0+r1| and not smaller |r0+r1|
    //circles are tangent
    m_vPoints[0] = m_pC0 + (m_r0/sumRad)*u;
    m_iNumIntersections = 1;
    m_iIntersectionType = TOUCHING;
  }

	return true;
}

//----------------------------------------------------------------------------
// Explicit instantiation.
//----------------------------------------------------------------------------
template
class CIntersectorCircleCircle<Real>;

//----------------------------------------------------------------------------
}
