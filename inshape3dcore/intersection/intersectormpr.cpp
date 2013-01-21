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

#include "intersectormpr.h"
#include <mymath.h>

namespace i3d {

template<class T>
CIntersectorMPR<T>::CIntersectorMPR()
{

}//end constructor

template<class T>
CIntersectorMPR<T>::CIntersectorMPR(const CConvexShape<T> &shape0, const CConvexShape<T> &shape1)
{
  m_pShape0 = &shape0;
  m_pShape1 = &shape1;
}

template<class T>
bool CIntersectorMPR<T>::Intersection()
{

  //Phase 1: find an initial portal
  FindInitialPortal();

  return false;

}//end Intersection

template<class T>
void CIntersectorMPR<T>::FindInitialPortal()
{

  //get the origin of the minkowski difference shape0.center - shape1.center
  CVector3<T> v = m_pShape0->GetCenter() - m_pShape1->GetCenter();

  //get the support point in the direction of -v
  CVector3<T> a = m_pShape0->GetSupport(-v) - m_pShape1->GetSupport(v);

  //check for collinearity

  CVector3<T> va = CVector3<T>::Cross(v,a);

  CVector3<T> b = m_pShape0->GetSupport(va) - m_pShape1->GetSupport(-va);

  CVector3<T> avbv = CVector3<T>::Cross(a-v,b-v);

  CVector3<T> c = m_pShape0->GetSupport(avbv) - m_pShape1->GetSupport(-avbv);


}//end FindInitialPortal

//----------------------------------------------------------------------------
// Explicit instantiation.
//----------------------------------------------------------------------------
template class CIntersectorMPR<Real>;

//----------------------------------------------------------------------------

}


