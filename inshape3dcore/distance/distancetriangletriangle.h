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
#ifndef DISTANCETRIANGLETRIANGLE_H
#define DISTANCETRIANGLETRIANGLE_H



//===================================================
//                     INCLUDES
//===================================================
#include <distance.h>
#include <iostream>
#include <vector>
#include <limits>
#include <triangle3.h>

namespace i3d {

/**
 * @brief Computes the distance between two triangles
 */  
template <typename T>
class CDistanceTriangleTriangle : public CDistance<T> {

public: 

  CDistanceTriangleTriangle(); 

  CDistanceTriangleTriangle(const Triangle3<T> &t0, const Triangle3<T> &t1) : m_Tri0(t0), m_Tri1(t1) {}; 

  ~CDistanceTriangleTriangle(); 

	T ComputeDistanceSqr();
	T ComputeDistance();

  Triangle3<T> m_Tri0;
  Triangle3<T> m_Tri1;

	T m_Tri0Param;
	T m_Tri1Param;

	//the point on segment 0
	using CDistance<T>::m_vClosestPoint0;
	//the point on segment 1
	using CDistance<T>::m_vClosestPoint1;

};

}
#endif
