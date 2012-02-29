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

#ifndef SEGMENT3_H
#define SEGMENT3_H

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

namespace i3d {

/** @brief A line segment in 3d space
 *
 * A line segment in 3d space
 */        
template <typename T>
class CSegment3
{
public:
	CSegment3(void);
	CSegment3(const CVector3<T> &vOrig, const CVector3<T> &vDir, T ext);
	CSegment3(const CVector3<T> &vP0, const CVector3<T> &vP1);
	CSegment3(const CSegment3<T> &copy);
	void CalcExtent();
	void CalcVertices(void);
	~CSegment3(void);

	CVector3<T> m_vDir;
	T m_Ext;
	CVector3<T> m_vP0;
	CVector3<T> m_vP1;
	CVector3<T> m_vCenter;

};

}

#endif
