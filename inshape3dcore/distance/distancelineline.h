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

#ifndef DISTANCELINELINE_H
#define DISTANCELINELINE_H

//===================================================
//					DEFINITIONS
//===================================================


//===================================================
//					INCLUDES
//===================================================
#include <iostream>
#include <vector>
#include <limits>
#include <vector3.h>
#include <line3.h>

namespace i3d {

/**
* @brief Computes the distance between two 3d lines
*
* Computes the distance between two 3d lines and the closest points.
*
*/    
template <typename T>
class CDistanceLineLine
{
public:
	CDistanceLineLine(void);
	CDistanceLineLine(const CLine3<T> &Line0, const CLine3<T> &Line1);
	~CDistanceLineLine(void);

	T ComputeDistanceSqr();
	T ComputeDistance();

	CLine3<T> m_Line0;
	CLine3<T> m_Line1;

	CVector3<T> m_vClosestPoint0;
	CVector3<T> m_vClosestPoint1;

	T m_Line0Param;
	T m_Line1Param;

};

}

#endif
