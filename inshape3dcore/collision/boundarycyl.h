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

#ifndef BOUNDARYCYL_H
#define BOUNDARYCYL_H


#include <aabb3.h>
#include <cylinder.h>
#include <shape.h>
#include <convexshape.h>
#include <rectangle3.h>
#include <boundarybox.h>

namespace i3d {

/**
* @brief A box-shaped boundary
* 
* A cylinder-shaped boundary for the simulation
*/
template <class T>
class BoundaryCyl : public Shape<T>
{
public:

  BoundaryCyl(void);

  BoundaryCyl(const Vector3<T> &vOrigin, const T extends[3]);

  ~BoundaryCyl(void);

  /**
   * The cylinder geometry of the boundary
   */
  Cylinder<T> cylinder_;

  AABB3<T> boundingBox_;

  AABB3<T> getAABB() { return boundingBox_; };

  T getVolume() const { return T(0.0); };

  Vector3<T> getCenter() const { return cylinder_.getCenter(); };

  bool isPointInside(const Vector3<T> &query) const { return false; };


};

/* typedefs to create float and double vectors */
typedef BoundaryCyl<double> BoundaryCyld;
typedef BoundaryCyl<float>  BoundaryCylf;
typedef BoundaryCyl<Real>   BoundaryCylr;

}

#endif
