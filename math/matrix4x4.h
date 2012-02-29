/***************************************************************************
 *   Copyright (C) 2006 by Raphael M�nster   *
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


//===================================================
//					DEFINITIONS
//===================================================
#if !defined _MATRIX4X4_H_
#define      _MATRIX4X4_H_


//===================================================
//					INCLUDES
//===================================================
#include <iostream>
#include "vector3.h"
#include "vector4.h"
#include "matrix3x3.h"

//===================================================
//			CLASS DESCRIPTION:
//			    a templated matrix class
//				mainly for use with the double
//				and float data type
//===================================================

namespace i3d {

/**
* @brief A 4x4 matrix
*
* A 4x4 matrix
*/    
template <class T>
class CMatrix4x4
{

public:
	
	/* use a union to allow different access methods */
	union
	{
		/* matrix entries as a one dimensional array */
		T m_Entries[16];

		/* a struct with an identifier for each entry */
		struct
		{
			T m_00; T m_01; T m_02; T m_03;
			T m_10; T m_11; T m_12; T m_13;
			T m_20; T m_21; T m_22; T m_23;
			T m_30; T m_31; T m_32; T m_33;
		};
	};

	CMatrix4x4(T nEntries[]);


	CMatrix4x4()
	{
		SetIdentity();
	};

	CMatrix4x4(T n00, T n01, T n02, T n03,
			   T n10, T n11, T n12, T n13,
			   T n20, T n21, T n22, T n23,
			   T n30, T n31, T n32, T n33)
	{

		m_00 = n00; m_01 = n01; m_02 = n02; m_03 = n03;
		m_10 = n10; m_11 = n11; m_12 = n12; m_13 = n13;
		m_20 = n20; m_21 = n21; m_22 = n22; m_23 = n23;
		m_30 = n30; m_31 = n31; m_32 = n32; m_33 = n33;

	}//end constructor

	bool GetInverseMatrix(CMatrix4x4 &matInverse) const;

//===================================================
//  			Determinate		
//===================================================

	T Determinate() const;
	
	inline T* Get() {return m_Entries;};


	inline void SetIdentity()
	{
		memset(m_Entries, 0, 16*sizeof(T));
		m_00 = 1;
		m_11 = 1;
		m_22 = 1;
		m_33 = 1;
	}//end SetIdentity

	inline void SetZero()
	{
		memset(m_Entries, 0, 16*sizeof(T));
	}//end SetIdentity

//===================================================
//  			Assignment operator		
//===================================================

	inline const CMatrix4x4& operator=(const CMatrix4x4 &rhs)
	{
		memcpy(m_Entries, rhs.m_Entries, 16*sizeof(T));
		return *this;
	}//end operator


	CVector4<T> operator * (const CVector4<T> &vVec) const;

	inline CMatrix4x4 GetTransposedMatrix()
	{

			   CMatrix4x4 mat(m_00, m_10, m_20, m_30,
    				          m_01, m_11, m_21, m_31,
	    					  m_02, m_12, m_22, m_32,
		        			  m_03, m_13, m_23, m_33);

		return mat; 

	}//end GetTransposedMatrix

	inline void TransposeMatrix()
	{
		CMatrix4x4 mat(m_00, m_10, m_20, m_30,
    	               m_01, m_11, m_21, m_31,
	    	           m_02, m_12, m_22, m_32,
		        	   m_03, m_13, m_23, m_33);

		memcpy(m_Entries, mat.m_Entries, 16*sizeof(T));

	}//end TransposeMatrix

	CMatrix3x3<T> GetSubMatrix3x3(int i, int j) const;
	
//===================================================
//  			output operator		
//===================================================
	template<typename Templateparm>
	friend std::ostream &operator << (std::ostream &out, const CMatrix4x4<Templateparm> &rhs);

};//end class


typedef CMatrix4x4<float> CMatrix4x4f;
typedef CMatrix4x4<double> CMatrix4x4d;

}

#endif