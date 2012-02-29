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

#ifndef DISTANCE_H
#define DISTANCE_H

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

/**
* @brief Base class for distance computations
*
* The base class for distance computations, the interface
* includes methods to compute either the squared distance or the real distance.
* Additionally the closest points between two objects are always computed.
* The contact set is computed if neccessary
*/
template<typename T>
class CDistance
{
public:
	CDistance(void);
	virtual ~CDistance(void);

	virtual T ComputeDistanceSqr();
	virtual T ComputeDistance();

	CVector3<T> m_vClosestPoint0;
	CVector3<T> m_vClosestPoint1;

};

}

#endif
