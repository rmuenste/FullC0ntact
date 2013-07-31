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

#ifndef _TRIANGLE3D_H
#define _TRIANGLE3D_H

//===================================================
//					DEFINITIONS
//===================================================


//===================================================
//					INCLUDES
//===================================================
#include <iostream>
#include <vector3.h>

namespace i3d {

/**
 * @brief A triangle class that stores the coordinates of the three vertices
 */
template<class T>
class CTriangle3
{

public:

	/* constructors */
	CTriangle3(void);

	CTriangle3(const CVector3<T> &vV0, const CVector3<T> &vV1, const CVector3<T> &vV2);

	CTriangle3(const CTriangle3 &triT);

	/* deconstructors */
	~CTriangle3(void){};

	inline CVector3<T> Get(int i)
	{

		if(i==0)return m_vV0;
		else if(i==1)return m_vV1;
		else if(i==2)return m_vV2;
		else return CVector3<T>();

	}//end Get

	inline CVector3<T> Get(int i) const
	{

		if(i==0)return m_vV0;
		else if(i==1)return m_vV1;
		else if(i==2)return m_vV2;
		else return CVector3<T>();

	}//end Get

	inline void operator=(const CTriangle3<T> &triT)
	{
		m_vV0 = triT.m_vV0;
		m_vV1 = triT.m_vV1;
		m_vV2 = triT.m_vV2;
	}//end operator=

	inline CVector3<T> GetCenter()
	{
		CVector3<T> vCenter = (m_vV0 + m_vV1 + m_vV2) * (1.0/3.0);
		return vCenter;
	};
  
  inline T GetArea()
  {
    T area = 0.5 * CVector3<T>::Cross((m_vV1 - m_vV0),(m_vV2 - m_vV0)).mag();
    return fabs(area);
  };
  
  inline CVector3<T> GetNormal()
  { 
    CVector3<T> normal = CVector3<T>::Cross((m_vV1 - m_vV0),(m_vV2 - m_vV0));
    normal.Normalize();
    return normal;      
  }

	CVector3<T> m_vV0;	
	CVector3<T> m_vV1;	
	CVector3<T> m_vV2;	
};

template<class T>
CTriangle3<T>::CTriangle3()
{
}//end constructor

template<class T>
CTriangle3<T>::CTriangle3(const CVector3<T> &vV0, const CVector3<T> &vV1, const CVector3<T> &vV2) 
:	m_vV0(vV0), m_vV1(vV1), m_vV2(vV2)
{

}//end constructor

template<class T>
CTriangle3<T>::CTriangle3(const CTriangle3<T> &triT)
{
	m_vV0 = triT.m_vV0;
	m_vV1 = triT.m_vV1;
	m_vV2 = triT.m_vV2;
}//end constructor

typedef CTriangle3<float> CTriangle3f;
typedef CTriangle3<double> CTriangle3d;
typedef CTriangle3<Real> CTriangle3r;

}

#endif
