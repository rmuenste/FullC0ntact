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

#ifndef RECTANGLE3_H
#define RECTANGLE3_H

//===================================================
//                   DEFINITIONS
//===================================================


//===================================================
//                    INCLUDES
//===================================================
#include <iostream>
#include <vector>
#include <limits>
#include <vector3.h>

namespace i3d {

/** @brief A rectangle in 3d space
 *
 * A rectangle in 3d space
 */   
template <typename T>
class Rectangle3
{
public:
  Rectangle3(void);

  ~Rectangle3(void);

  Rectangle3(const Rectangle3<T> &copy);

  Rectangle3(const CVector3<T>& vCenter, const CVector3<T> vUV[2],const T Extents[2]);

  Rectangle3(const CVector3<T>& vCenter, const CVector3<T>& vU,const CVector3<T>& vV, T Extent0, T Extent1);

  void computeVertices (CVector3<T> vVertices[4]) const;

  CVector3<T> getNormal() const {return CVector3<T>::Cross(uv_[0],uv_[1]);};

  CVector3<T> center_;
  CVector3<T> uv_[2];
  T extents_[2];
};

}

#endif
