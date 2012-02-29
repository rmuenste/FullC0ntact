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

#ifndef DISTANCESEGREC_H
#define DISTANCESEGREC_H

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
#include <segment3.h>
#include <rectangle3.h>


namespace i3d {

/**
* @brief Computes the distance between a line segment and a rectangle in 3d
*
* Computes the distance between a line segment and a rectangle in 3d
*
*/  
template <typename T>
class CDistanceSegRec : public CDistance<T>
{
public:
	CDistanceSegRec(void);
	~CDistanceSegRec(void);

	CDistanceSegRec(const CSegment3<T>& seg, const CRectangle3<T>& rec);

	T ComputeDistanceSqr();
	T ComputeDistance();

	CSegment3<T>   m_Seg;
	CRectangle3<T> m_Rec;

	T m_ParamSegment;
	T m_ParamRectangle[2];

	//the closest point on the segment
	using CDistance<T>::m_vClosestPoint0;

	//the closest point on the rectangle
	using CDistance<T>::m_vClosestPoint1;

};

}

#endif
