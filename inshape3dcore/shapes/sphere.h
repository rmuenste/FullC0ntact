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

#ifndef SPHERE_H_
#define SPHERE_H_

//===================================================
//					INCLUDES
//===================================================
#include <limits>
#include <vector3.h>
#include <shape.h>
#include <convexshape.h>

namespace i3d {

/** @brief A class for a sphere
 *
 * A class for a sphere with various sphere-related operations
 */      
template<class T>
class Sphere : public ConvexShape<T>
{
public:
	Sphere(void);

	Sphere(const CVector3<T> &vCenter, T dRad): center_(vCenter), radius_(dRad)
	{
	}//end

	~Sphere(void);

	Sphere(const Sphere<T> &copy)
	{
		this->radius_ = copy.radius_;
		this->center_ = copy.center_;
	}

	inline CVector3<T> eval(T phi, T theta) const
	{
	  return CVector3<T>(center_.x+radius_*cos(phi)*cos(theta), 
						 center_.y+radius_*cos(phi)*sin(theta), 
						 center_.z+radius_*sin(phi));
	}

/** 
 *
 * Returns the radius of the sphere
 * @return Returns the radius
 */
	inline T getRadius() {return radius_;};

/** 
 *
 * Returns the radius of the sphere const version
 * @return Returns the radius
 */
	inline T getRadius() const {return radius_;};

/**
 *
 * Returns the center of the sphere
 * @return Returns the center of the sphere
 */
	inline CVector3<T>& getCenter() {return center_;};

/**
 *
 * Returns the center of the sphere
 * @return Returns the center of the sphere
 */
	inline CVector3<T> getCenter() const {return center_;};

/** 
 *
 * Returns the volume of the sphere
 * @return Returns the volume of the sphere
 */
	inline T getVolume() const {return (T)4.0/(T)3.0*3.14*radius_*radius_*radius_;};

/**
 * Returns a bounding box for the sphere
 * @return The axis-aligned bounding box of the sphere
 */
	AABB3<T> getAABB();

  CVector3<T> getSupport(const CVector3<T> &v) const
  {
    if(v.mag() < CMath<T>::TOLERANCEZERO)
      return CVector3<T>(radius_,0,0);
    else
      return center_ + radius_ * (v*(1.0/v.mag()));
  };

  CVector3<T> getPointOnBoundary() const
  {
    return center_ + CVector3<T>(radius_,0,0);
  };

/**
 * Returns whether if query point is inside
 * @return Returns true when the query point is inside
 */
  bool isPointInside(const CVector3<T> &vQuery) const
  {
    CVector3<T> diff = (vQuery - center_);
    return (diff * diff <= radius_ * radius_); 
  }
  
  CVector3<T> center_;
  T           radius_;

};



typedef Sphere<float> Spheref;
typedef Sphere<double> Sphered;
typedef Sphere<Real> Spherer;

}

#endif
