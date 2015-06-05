/*
   This library is free software; you can redistribute it and/or
   modify it under the terms of the GNU Library General Public
   License version 2 as published by the Free Software Foundation.

   This library is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
   Library General Public License for more details.

   You should have received a copy of the GNU Library General Public License
   along with this library; see the file COPYING.LIB.  If not, write to
   the Free Software Foundation, Inc., 51 Franklin Street, Fifth Floor,
   Boston, MA 02110-1301, USA.
*/

#ifndef _INTERSECRAYTRI3_H
#define _INTERSECRAYTRI3_H

//===================================================
//					DEFINITIONS
//===================================================


//===================================================
//					INCLUDES
//===================================================
#include <iostream>
#include "ray3.h"
#include "triangle3.h"

namespace i3d {

/**
* @brief Computes whether a ray and a triangle intersect
*
* Computes whether a ray and a triangle intersect
*
*/      
template<class T>
class CIntersectorRay3Tri3
{

public:

	/* constructors */
	CIntersectorRay3Tri3(const Ray3<T> &rRay, const Triangle3<T> &trTriangle);

	/* deconstructors */
	~CIntersectorRay3Tri3(void){};



	/* member functions */
	bool Intersection();

  /* member functions */
  bool Intersection2();

private:

	const Ray3<T> *m_rRay;
	const Triangle3<T> *m_trTriangle;

	 T m_fRayT, m_fTriB0, m_fTriB1, m_fTriB2;


};

typedef CIntersectorRay3Tri3<float> CIntersectorRay3Tri3f;
typedef CIntersectorRay3Tri3<double> CIntersectorRay3Tri3d;

}

#endif