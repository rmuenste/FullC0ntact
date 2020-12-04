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
#ifndef DISTANCEAABBAABB_H
#define DISTANCEAABBAABB_H



//===================================================
//                     INCLUDES
//===================================================
#include <aabb3.h>

namespace i3d {

/**
* @brief This class calculates the distance between two AABBs
* 
*/
template <typename T>
class CDistanceAabbPoint {

public: 

CDistanceAabbPoint(const AABB3<T> &rAABB1, const Vector3<T> &vector); 

~CDistanceAabbPoint();

T ComputeDistanceSqr();

T ComputeDistance();

const AABB3<T> *m_pAABB1;
const Vector3<T>* point_;

};

}
#endif
