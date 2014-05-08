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
#include "cone.h"

namespace i3d {

template<class T>
Cone<T>::Cone() 
{

}

template<class T>
Cone<T>::~Cone() 
{

}

template <class T>
Vector3<T> Cone<T>::getSupport(const Vector3<T> &v) const
{
  T uv;
  Vector3<T> delta;
  Vector3<T> support;
  T length = 2*m_dHalfLength;
  T normv  = v.mag();

  T anglealpha = sin(m_dRadius/sqrt(m_dRadius*m_dRadius+length*length));

  T sign = (uv=m_vU*v) > 0 ? T(1.0) : T(-1.0);

  if(anglealpha <= uv/normv)
  {
    return m_vCenter + m_vU * m_dHalfLength;
  }
  else
  {
    Vector3<T> w  = v - (uv)*m_vU;    
    T normw = w.mag();
    if(normw < CMath<T>::TOLERANCEZERO)
    {
      return m_vCenter - m_dHalfLength * m_vU + m_dRadius * (w*(1.0/normw));
    }
    else
      return m_vCenter - m_dHalfLength * m_vU;
  }
}

//----------------------------------------------------------------------------
// Explicit instantiation.
//----------------------------------------------------------------------------
template class Cone<Real>;

//template class CCone<double>;
//----------------------------------------------------------------------------
}
