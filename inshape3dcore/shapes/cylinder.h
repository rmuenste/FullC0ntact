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
#ifndef CYLINDER_H
#define CYLINDER_H



//===================================================
//                     INCLUDES
//===================================================
#include <convexshape.h>
#include <vector3.h>
#include <aabb3.h>

namespace i3d {

/** @brief A cylinder shape that can be used in the rigid body simulation
*
* A cylinder shape that can be used in the rigid body simulation
*/    
template <class T>
class Cylinder : public ConvexShape<T> {

public: 

Cylinder(const CVector3<T> &center, const CVector3<T> u, T radius, T h2) : center_(center),
          u_(u), radius_(radius), halfLength_(h2) {};

Cylinder();

~Cylinder(); 

/**
 * @see CConvexShape::GetSupport()
 * 
 */
CVector3<T> getSupport(const CVector3<T> &v) const;

/**
 * @see CConvexShape::GetPointOnBoundary
 */
CVector3<T> getPointOnBoundary() const;

/**
 * @see CShape::GetAABB
 */
CAABB3<T> getAABB(){ T extents[3]={radius_,radius_,halfLength_};return CAABB3<T>(center_,extents);};

/**
 * @see CShape::Volume
 */
T getVolume() const {return CMath<T>::SYS_PI * radius_ * radius_ * (2.0 * halfLength_);};


inline CVector3<T> eval(T phi) const
{
  return CVector3<T>(center_.x+radius_*cos(phi), center_.y+radius_*sin(phi),0);
}

/**
 * @see CShape::PointInside
 */
bool isPointInside(const CVector3<T> &vQuery) const
{
  if((vQuery.z > halfLength_) || (vQuery.z < -halfLength_))
  {
    return false;     
  }
  
  T dist = (vQuery.x*vQuery.x) + (vQuery.y*vQuery.y);
  
  if(dist > (radius_*radius_))
  {
    return false;
  }
  
  return true;
}  

inline CVector3<T> getCenter() const {return center_;};

inline CVector3<T> getU() const {return u_;};

inline T getRadius() const {return radius_;};

inline T getHalfLength() const {return halfLength_;};

inline void setCenter(const CVector3<T> &center) {center_=center;};

inline void setU(const CVector3<T> &u) {u_=u;};

inline void setRadius(T radius) {radius_=radius;};

inline void setHalfLength(T halfLength) {halfLength_=halfLength;};

private:

  CVector3<T> center_;
  CVector3<T> u_;
  T           radius_;
  T           halfLength_;

};

typedef Cylinder<float> Cylinderf;
typedef Cylinder<double> Cylinderd;
typedef Cylinder<Real> Cylinderr;

}
#endif
