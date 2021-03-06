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

#ifndef DISTANCEOBBPLANE_H
#define DISTANCEOBBPLANE_H

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
#include <rigidbody.h>
#include "distance.h"

namespace i3d {

/**
* @brief Computes the distance between an OBBs and a plane in 3d
*
* Computes the distance between an OBBs and a plane in 3d
*
*/   
template <typename T>
class CDistanceOBB3Plane :
	public CDistance<T>
{
public:

	CDistanceOBB3Plane(void) = default;
	~CDistanceOBB3Plane(void);

	CDistanceOBB3Plane(RigidBody *pBody,const Vector3<T> &vPoint,const Vector3<T> &vNormal) :
		m_pBody(pBody), m_vPoint(vPoint), m_vNormal(vNormal), m_pBox(nullptr)
	{

	};

	CDistanceOBB3Plane(OBB3<T> &pBox,const Vector3<T> &vPoint,const Vector3<T> &vNormal) :
	m_pBody(nullptr), m_vPoint(vPoint), m_vNormal(vNormal), m_pBox(&pBox)
	{

	};

	T ComputeDistanceSqr();
	T ComputeDistance();

	//store a pointer to the rigid body
	RigidBody *m_pBody;

	//store the plane
	Vector3<T> m_vPoint;

	Vector3<T> m_vNormal;

	//store the box
	OBB3<T> *m_pBox;

	//point on the plane
	using CDistance<T>::m_vClosestPoint0;
	//point on the box
	using CDistance<T>::m_vClosestPoint1;

};

}

#endif
