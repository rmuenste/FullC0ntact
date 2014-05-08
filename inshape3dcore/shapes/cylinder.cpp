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
Cylinder<T>::Cylinder() 
{

}

template <class T>
Cylinder<T>::~Cylinder() 
{

}

template <class T>
Vector3<T> Cylinder<T>::getSupport(const Vector3<T> &v) const
{
  T uv;
  Vector3<T> delta;
  Vector3<T> support;    
  T sign = (uv=u_*v) > 0 ? T(1.0) : T(-1.0);
  
  Vector3<T> w  = v - (uv)*u_;

  
  if((w*w)  < CMath<T>::TOLERANCEZERO)
    delta = sign * halfLength_ * u_;
  else
  {
    w.Normalize();
    delta = sign * halfLength_ * u_ + radius_ * w;
  }
      
  return (center_ + delta);
}

template <class T>
Vector3<T> Cylinder<T>::getPointOnBoundary() const
{
  //return top
  return center_ + halfLength_ * u_;
}

//----------------------------------------------------------------------------
// Explicit instantiation.
//----------------------------------------------------------------------------
template class Cylinder<float>;

template class Cylinder<double>;

//template class CCylinder<double>;
//----------------------------------------------------------------------------

}

