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

#ifndef RECTANGLE3_H
#define RECTANGLE3_H

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

/** @brief A rectangle in 3d space
 *
 * A rectangle in 3d space
 */   
template <typename T>
class CRectangle3
{
public:
	CRectangle3(void);

	~CRectangle3(void);

	CRectangle3(const CRectangle3<T> &copy);

	CRectangle3(const CVector3<T>& vCenter, const CVector3<T> vUV[2],const T Extents[2]);

	CRectangle3(const CVector3<T>& vCenter, const CVector3<T>& vU,const CVector3<T>& vV, T Extent0, T Extent1);

	void ComputeVertices (CVector3<T> vVertices[4]) const;

  CVector3<T> GetNormal() const {return CVector3<T>::Cross(m_vUV[0],m_vUV[1]);};

	//// Get the rectangle corners.
	//CVector3<T> GetPPCorner () const;  // C + e0*A0 + e1*A1
	//CVector3<T> GetPMCorner () const;  // C + e0*A0 - e1*A1
	//CVector3<T> GetMPCorner () const;  // C - e0*A0 + e1*A1
	//CVector3<T> GetMMCorner () const;  // C - e0*A0 - e1*A1

	CVector3<T> m_vCenter;
	CVector3<T> m_vUV[2];
	T m_Extents[2];
};

}

#endif
