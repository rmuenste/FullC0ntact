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

#include "intersectorspheresegment.h"

namespace i3d {

template <class T>
bool CIntersectorSphereSegment<T>::Intersection()
{
  CVector3<T> diff = m_pSegment->m_vCenter - m_pSphere->center_;

  if((diff * diff < CMath<T>::TOLERANCEZERO) && (fabs(m_pSegment->m_Ext - m_pSphere->radius_) < CMath<T>::TOLERANCEZERO))
  {
    m_vPoints[0] = m_pSegment->m_vCenter + m_pSegment->m_Ext * m_pSegment->m_vDir;
    m_vPoints[1] = m_pSegment->m_vCenter - m_pSegment->m_Ext * m_pSegment->m_vDir;
    m_iNumIntersections = 2;
    m_iIntersectionType = SEGMENT;
    return true;
  }

  T a0 = diff * diff - m_pSphere->radius_ * m_pSphere->radius_;
  T a1 = m_pSegment->m_vDir * diff;
  T discr = a1*a1 - a0;
  if (discr < (T)0)
  {
      m_iNumIntersections = 0;
      return false;
  }

  T tmp0 = m_pSegment->m_Ext*m_pSegment->m_Ext + a0;
  T tmp1 = ((T)2)*a1*m_pSegment->m_Ext;
  T qm = tmp0 - tmp1;
  T qp = tmp0 + tmp1;
  T root;
  if (qm*qp <= (T)0)
  {
      root = sqrt(discr);
      m_dParameterSegment[0] = (qm > (T)0 ? -a1 - root : -a1 + root);
      m_vPoints[0] = m_pSegment->m_vCenter + m_dParameterSegment[0] * m_pSegment->m_vDir;
      m_iNumIntersections = 1;
      m_iIntersectionType = POINT;
      return true;
  }

  if (qm > (T)0 && fabs(a1) < m_pSegment->m_Ext)
  {
    if (discr >= CMath<T>::TOLERANCEZERO)
      {
          root = sqrt(discr);
          m_dParameterSegment[0] = -a1 - root;
          m_dParameterSegment[1] = -a1 + root;
          m_vPoints[0] = m_pSegment->m_vCenter + m_dParameterSegment[0] *
              m_pSegment->m_vDir;
          m_vPoints[1] = m_pSegment->m_vCenter + m_dParameterSegment[1] *
              m_pSegment->m_vDir;
          m_iNumIntersections = 2;
          m_iIntersectionType = SEGMENT;
      }
      else
      {
          m_dParameterSegment[0] = -a1;
          m_vPoints[0] = m_pSegment->m_vCenter + m_dParameterSegment[0] *
              m_pSegment->m_vDir;
          m_iNumIntersections = 1;
          m_iIntersectionType = POINT;
      }
  }
  else
  {
      m_iNumIntersections = 0;
      m_iIntersectionType = EMPTY;
  }

  return m_iNumIntersections > 0;
}

//----------------------------------------------------------------------------
// Explicit instantiation.
//----------------------------------------------------------------------------
template
class CIntersectorSphereSegment<Real>;

//----------------------------------------------------------------------------
}
