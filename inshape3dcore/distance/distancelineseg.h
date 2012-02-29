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

#ifndef DISTANCELINESEG_H
#define DISTANCELINESEG_H

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
#include <rectangle3.h>
#include <segment3.h>
#include <distance.h>
#include <line3.h>

namespace i3d {

/**
* @brief Computes the distance between a line and a line segment in 3d space
*
* Computes the distance between a line and a line segment in 3d space
*
*/   
template <typename T>
class CDistanceLineSeg : public CDistance<T>
{
	public:
	CDistanceLineSeg(void);
	CDistanceLineSeg(const CLine3<T>& line, const CSegment3<T>& seg);
	~CDistanceLineSeg(void);

	T ComputeDistanceSqr();
	T ComputeDistance();

	CLine3<T>      m_Line;
	CSegment3<T>   m_Seg;

	T m_ParamLine;
	T m_ParamSegment;
	
	using CDistance<T>::m_vClosestPoint0;
	using CDistance<T>::m_vClosestPoint1;
	
};

}

#endif // DISTANCELINESEG_H
