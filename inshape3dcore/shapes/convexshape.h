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
#ifndef CONVEXSHAPE_H
#define CONVEXSHAPE_H



//===================================================
//                     INCLUDES
//===================================================
#include <vector3.h>
#include <shape.h>

namespace i3d {

/**
* @brief A base class for convex shapes that can be used in the rigid body simulation
* 
* A base class for convex shapes that can be used in the rigid body simulation
*/
template <class T>
class CConvexShape : public CShape<T>{

public: 

CConvexShape(); 

virtual ~CConvexShape(); 

/**
 * The objects of call CConvexShape must implement the GetSupport interface,
 * so that we can use the GJK-algorithm for distance calculations.
 */
virtual CVector3<T> GetSupport(const CVector3<T> &v) const = 0;

/**
 * In order to use the GJK-algorithm we have to start with a point on the
 * boundary of the shape, so each CConvexShape must have a method to
 * compute such a point.
 */
virtual CVector3<T> GetPointOnBoundary() const = 0;

};

typedef CConvexShape<float> CConvexShapef;
typedef CConvexShape<double> CConvexShaped;
typedef CConvexShape<Real> CConvexShaper;

}
#endif
