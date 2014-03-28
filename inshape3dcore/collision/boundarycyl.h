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
class BoundaryCyl : public BoundaryBox<T>
{
public:

  BoundaryCyl(void);

  BoundaryCyl(const CVector3<T> &vOrigin, const T extends[3]);

  ~BoundaryCyl(void);

  /**
   * The cylinder geometry of the boundary
   */
  Cylinder<T> m_Cylinder;

};

/* typedefs to create float and double vectors */
typedef BoundaryCyl<double> BoundaryCyld;
typedef BoundaryCyl<float>  BoundaryCylf;
typedef BoundaryCyl<Real>   BoundaryCylr;

}

#endif
