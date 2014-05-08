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

#include "segment3.h"

namespace i3d {

template <typename T>
Segment3<T>::Segment3(void)
{
  
}

template <typename T>
Segment3<T>::Segment3(const Segment3<T> &copy)
{
  ext_      = copy.ext_;
  dir_      = copy.dir_;
  p0_       = copy.p0_;
  p1_       = copy.p1_;
  center_   = copy.center_;
}

template <typename T>
Segment3<T>::~Segment3(void)
{
  
}

template <typename T>
Segment3<T>::Segment3(const Vector3<T> &vOrig, const Vector3<T> &vDir, T ext)
{
  center_ = vOrig;
  dir_    = vDir;
  ext_     = ext;
  calcVertices();
}

template <typename T>
Segment3<T>::Segment3(const Vector3<T> &vP0, const Vector3<T> &vP1)
{
  p0_ = vP0;
  p1_ = vP1;
  calcExtent();
}

template <typename T>
void Segment3<T>::calcExtent(void)
{
  center_ = ((T)0.5)*(p0_ + p1_);
  dir_ = p1_ - p0_;
  ext_ = ((T)0.5)*dir_.mag();
  dir_.Normalize();
}

template <typename T>
void Segment3<T>::calcVertices(void)
{
  p0_=center_ - dir_ * ext_;
  p1_=center_ + dir_ * ext_;
}
  
//----------------------------------------------------------------------------
// Explicit instantiation.
//----------------------------------------------------------------------------
template class Segment3<float>;

template class Segment3<double>;
//----------------------------------------------------------------------------

}