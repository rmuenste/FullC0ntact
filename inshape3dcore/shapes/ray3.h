/***************************************************************************
 *   Copyright (C) 2006 by Raphael Mnster   *
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

#ifndef _RAY3D_H
#define _RAY3D_H

//===================================================
//					DEFINITIONS
//===================================================


//===================================================
//					INCLUDES
//===================================================
#include <iostream>
#include <vector3.h>

namespace i3d {

/** @brief A ray in 3d space
 *
 * A ray in 3d space
 */     
template <class T>
class Ray3
{
public:

	/* constructor */
	Ray3(void);

	//pass the origin and direction as parameters
	Ray3(const Vector3<T> &vOrig, const Vector3<T> &vDir);

	/* deconstructor */
	~Ray3(void){};

	//member variables for origin and direction
	Vector3<T> m_vOrig;
	Vector3<T> m_vDir;

};

template<class T>
Ray3<T>::Ray3()
{
}//end constructor

template<class T>
Ray3<T>::Ray3(const Vector3<T> &vOrig, const Vector3<T> &vDir) : m_vOrig(vOrig), m_vDir(vDir) 
{
}//end constructor

typedef Ray3<float> Ray3f;
typedef Ray3<double> Ray3d;
typedef Ray3<Real> Ray3r;

}

#endif
