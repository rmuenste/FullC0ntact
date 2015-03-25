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


#if !defined(_CMATRIX2X2_H)
#define _CMATRIX2X2_H

//===================================================
//					INCLUDES
//===================================================

#include "vector2.h"
#include "vector3.h"
#include <string.h>
namespace i3d {

/**
* @brief A 2x2 matrix
*
* A 2x2 matrix
*/  
template<class T>
class CMatrix2x2 
{
public:
	CMatrix2x2(T m00, T m01, T m10, T m11);
	CMatrix2x2(T entries[]);
	~CMatrix2x2(){};
	


	union
	{
		T m_dEntries[4];

		struct
		{
			T m_d00;
			T m_d10;
			T m_d01;
			T m_d11;
		};
	};

	inline CVector2<T> operator *(const CVector2<T> &rhs)
	{
		CVector2<T> res(m_d00*rhs.x + m_d01 * rhs.y, m_d10 * rhs.x + m_d11 * rhs.y);
		return res;
	}//end operator *

	inline CMatrix2x2<T>& operator +(const CVector2<T> &rhs)
	{
		m_d00 += rhs.x;
		m_d10 += rhs.y;
		m_d01 += rhs.x;
		m_d11 += rhs.y;

		return *this;
	}//end operator

	inline T GetDeterminate()
	{
		return (m_d00*m_d11)-(m_d10*m_d01);
	}//end GetDeterminate

	inline void TransposeMatrix()
	{
		CMatrix2x2 mat(m_d00, m_d10, m_d01, m_d11);
		memcpy(m_dEntries, mat.m_dEntries, 4 * sizeof(T));

	}//end TransposeMatrix

	inline CMatrix2x2 GetTransposedMatrix()
	{
		CMatrix2x2 mat(m_d00, m_d10, m_d01, m_d11);
		return mat;
	}//end GetTransposedMatrix

	inline void operator=(const CMatrix2x2 &rhs)
	{
		memcpy(m_dEntries, rhs.m_dEntries, 4 * sizeof(T));
	}//end operator=

	inline CMatrix2x2<T>& operator -(const CVector2<T> &rhs)
	{
		m_d00 -= rhs.x;
		m_d10 -= rhs.y;
		m_d01 -= rhs.x;
		m_d11 -= rhs.y;

		return *this;
	}//end operator
	template<typename Templateparm>
	friend std::ostream& operator<<(std::ostream& out, const CMatrix2x2<Templateparm> &v1); 

};

typedef CMatrix2x2<double> CMatrix2x2d;
typedef CMatrix2x2<float>  CMatrix2x2f;

typedef CMatrix2x2<Real> MATRIX2X2;

#define IJ2(i,j) (i*2 + j)

}

#endif

