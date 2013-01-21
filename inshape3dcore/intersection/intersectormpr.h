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

#ifndef _INTERSECMPR_H
#define _INTERSECMPR_H

//===================================================
//					DEFINITIONS
//===================================================


//===================================================
//					INCLUDES
//===================================================
#include <iostream>
#include <convexshape.h>

namespace i3d {

/**
* @brief Computes whether two convex objects intersect
*
* Computes whether two convex objects intersect
*
*/      
template<class T>
class CIntersectorMPR
{

public:

	/* constructors */
	CIntersectorMPR();

  CIntersectorMPR(const CConvexShape<T> &shape0, const CConvexShape<T> &shape1);

	/* deconstructors */
	~CIntersectorMPR(void){};

	/* member functions */
	bool Intersection();

private:

  void FindInitialPortal();

  const CConvexShape<T> *m_pShape0;
  const CConvexShape<T> *m_pShape1; 

};

typedef CIntersectorMPR<float> CIntersectorMPRf;
typedef CIntersectorMPR<double> CIntersectorMPRd;

}

#endif