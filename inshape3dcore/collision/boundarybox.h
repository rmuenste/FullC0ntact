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

#ifndef BOUNDARYBOX_H
#define BOUNDARYBOX_H


#include <aabb3.h>
#include <shape.h>
#include <convexshape.h>
#include <rectangle3.h>

namespace i3d {

/**
* @brief A box-shaped boundary
* 
* A box-shaped boundary for the simulation
*/
template <class T>
class BoundaryBox : public Shape<T>
{
public:

  BoundaryBox(void);
  
  BoundaryBox(const Vector3<T> &vOrigin, const T extends[3]);

  ~BoundaryBox(void);

  AABB3<T> getAABB() {return boundingBox_;};

  T getVolume() const {return T(boundingBox_.getVolume());};

  void translateTo(const Vector3<T> &vPos)
  {

  };

  /**
   * Returns the geometric center of the shape
   *
   */
  Vector3<T> getCenter() const {return boundingBox_.getCenter();};

  bool isPointInside(const Vector3<T> &query) const
  {
    //TODO:implement 
    return false; 
  }  

  void calcValues();
  
  AABB3<T> boundingBox_;
  
  T extents_[6];

  Vector3<T> normals_[6];
  Vector3<T> points_[6];

  std::vector< Rectangle3<T> > m_vBorders;
  
};

/* typedefs to create float and double vectors */
typedef BoundaryBox<double> BoundaryBoxd;
typedef BoundaryBox<float>  BoundaryBoxf;
typedef BoundaryBox<Real>   BoundaryBoxr;

}

#endif
