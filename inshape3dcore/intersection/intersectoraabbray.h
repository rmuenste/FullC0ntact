/***************************************************************************
 *   Copyright (C) 2006 by Raphael Muenster   *
 *   raphael@Cortez   *
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   You should have received a copy of the GNU General Public License     *
 *   along with this program; if not, write to the                         *
 *   Free Software Foundation, Inc.,                                       *
 *   59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.             *
 ***************************************************************************/

#ifdef WIN32
#pragma once
#endif

#ifndef _INTERSECRAYAABB3_H
#define _INTERSECRAYAABB3_H

//===================================================
//					DEFINITIONS
//===================================================


//===================================================
//					INCLUDES
//===================================================
#include <limits>
#include <iostream>
#include "ray3.h"
#include "aabb3.h"

namespace i3d {

/**
* @brief Computes whether a ray and an AABB intersect
*
* Computes whether a ray and an AABB intersect
*
*/    
template<class T>
class CIntersectorAABBRay3
{

public:

	/* constructors */
	CIntersectorAABBRay3(const CRay3<T> &rRay, const AABB3<T> &bxAABB3);

	/* deconstructors */
	~CIntersectorAABBRay3(void){};



	/*!
	* We calculate if the given ray intersects with the given aabb
	*/
	bool Intersection();

private:

	const CRay3<T> *m_rRay;
	const AABB3<T> *m_bxAABB3;

};

typedef CIntersectorAABBRay3<float> CIntersectorAABBRay3f;
typedef CIntersectorAABBRay3<double> CIntersectorAABBRay3d;

}

#endif
