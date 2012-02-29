/*
    <one line to give the program's name and a brief idea of what it does.>
    Copyright (C) <2011>  <Raphael Muenster>

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


//===================================================
//                     INCLUDES
//===================================================
#include "cylinder.h"
#include <mymath.h>

namespace i3d {

template <class T>
CCylinder<T>::CCylinder() 
{

}

template <class T>
CCylinder<T>::~CCylinder() 
{

}

template <class T>
CVector3<T> CCylinder<T>::GetSupport(const CVector3<T> &v) const
{
  T uv;
  CVector3<T> delta;
  CVector3<T> support;    
  T sign = (uv=m_vU*v) > 0 ? T(1.0) : T(-1.0);
  
  CVector3<T> w  = v - (uv)*m_vU;

  
  if((w*w)  < CMath<T>::TOLERANCEZERO)
    delta = sign * m_dHalfLength * m_vU;
  else
  {
    w.Normalize();
    delta = sign * m_dHalfLength * m_vU + m_dRadius * w;
  }
      
  return (m_vCenter + delta);
}

template <class T>
CVector3<T> CCylinder<T>::GetPointOnBoundary() const
{
  //return top
  return m_vCenter + m_dHalfLength * m_vU;
}

//----------------------------------------------------------------------------
// Explicit instantiation.
//----------------------------------------------------------------------------
template class CCylinder<Real>;

//template class CCylinder<double>;
//----------------------------------------------------------------------------

}

