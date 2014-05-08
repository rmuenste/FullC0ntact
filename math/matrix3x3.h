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

#if !defined(_CMATRIX3X3_H)
#define _CMATRIX3X3_H

//===================================================
//					INCLUDES
//===================================================
#include <iostream>
#include "vector3.h"
#include <cstring>

namespace i3d {

/**
* @brief A 3x3 matrix
*
* A 3x3 matrix
*/    
template<class T>
class CMatrix3x3 
{
public:
	CMatrix3x3();
	CMatrix3x3( T m00,  T m01,  T m02,  T m10,  T m11,  T m12,
					    T m20,  T m21,  T m22);
	CMatrix3x3( T entries[]);

	CMatrix3x3(const Vector3<T> &c1,const Vector3<T> &c2,const Vector3<T> &c3);

	CMatrix3x3(const CMatrix3x3 &copy);

	~CMatrix3x3(){};

//===================================================
//  		 Matrix-Vector-Product	   
//===================================================
	Vector3<T> operator*(const Vector3<T> &rhs) const;

	void MatrixFromAngles(const Vector3<T> &vRotXYZ);

//===================================================
//  		 Matrix-Matrix-Product	   
//===================================================

	CMatrix3x3<T> operator*(const CMatrix3x3<T> &rhs) const;
	
//===================================================
//  		 Inverse Matrix
//===================================================

	CMatrix3x3<T> Inverse() const;
	
  static CMatrix3x3<T> GetSkewMatrix(const Vector3<T> &vector)
  {
	  return CMatrix3x3(0,-vector.z,vector.y,
                      vector.z,0,-vector.x,
					            -vector.y,vector.x,0);
  }

  static CMatrix3x3<T> GenIdentity()
  {
	  return CMatrix3x3(1,0,0,
                      0,1,0,
                      0,0,1);
  }
   
	union
	{
		T m_dEntries[9];

		struct
		{
			T m_d00;
			T m_d01;
			T m_d02;
			T m_d10;
			T m_d11;
			T m_d12;
			T m_d20;
			T m_d21;
			T m_d22;
		};
	};

/**
*  Set to the identity matrix
*/
	inline void SetIdentity()
	{
		memset(m_dEntries, 0, 9*sizeof(T));
		m_d00 = 1;
		m_d11 = 1;
		m_d22 = 1;
	}//end SetIdentity

/**
*  Set to the zero matrix
*/
	inline void SetZero()
	{
		memset(m_dEntries, 0, 9*sizeof(T));
	}//end SetIdentity

/**
*  Transpose the matrix
*/
	inline CMatrix3x3 GetTransposedMatrix()
	{
	   CMatrix3x3 mat(m_d00, m_d10, m_d20,
    				  m_d01, m_d11, m_d21,
	    			  m_d02, m_d12, m_d22);
		return mat;
	}//end GetTransposedMatrix

/**
*  Normalize column vectors
*/
	inline void Normalize()
	{
  
    Vector3<T> v0(m_dEntries[0],m_dEntries[1],m_dEntries[2]);
    Vector3<T> v1(m_dEntries[3],m_dEntries[4],m_dEntries[5]);
    Vector3<T> v2(m_dEntries[6],m_dEntries[7],m_dEntries[8]);
    v0.Normalize();
    v1.Normalize();
    v2.Normalize();
    m_dEntries[0]=v0.x;
    m_dEntries[1]=v0.y;
    m_dEntries[2]=v0.z;
    m_dEntries[3]=v1.x;
    m_dEntries[4]=v1.y;
    m_dEntries[5]=v1.z;
    m_dEntries[6]=v2.x;
    m_dEntries[7]=v2.y;
    m_dEntries[8]=v2.z;

	}//end GetTransposedMatrix


/**
*  Transpose the matrix
*/
	inline void TransposeMatrix()
	{
		CMatrix3x3 mat(m_d00, m_d10, m_d20,
    	               m_d01, m_d11, m_d21,
	    	           m_d02, m_d12, m_d22);

		memcpy(m_dEntries, mat.m_dEntries, 9*sizeof(T));

	}//end TransposeMatrix

/**
*  Calculate the Determinate
*  @return The determinate of the matrix
*/
	inline T Determinate()
	{
		T det = m_d00 * (m_d11*m_d22 - m_d21*m_d12) - m_d01 * (m_d10*m_d22 - m_d20*m_d12) + m_d02 * (m_d10*m_d21 - m_d20*m_d11);
		return det;
	}//end Determinate

//===================================================
//  			   Determinate		
//===================================================

	inline T Determinate() const
	{
		T det = m_d00 * (m_d11*m_d22 - m_d21*m_d12) - m_d01 * (m_d10*m_d22 - m_d20*m_d12) + m_d02 * (m_d10*m_d21 - m_d20*m_d11);
		return det;
	}//end Determinate

/**
*  Assigns the matrix rhs to the matrix on the left of the equals sign
*/
	inline void operator=(const CMatrix3x3 &rhs)
	{
		memcpy(m_dEntries, rhs.m_dEntries, 9 * sizeof(T));
	}//end operator=

	inline CMatrix3x3 operator+(const CMatrix3x3 rhs) const
	{
		return CMatrix3x3(m_d00+rhs.m_d00,m_d01+rhs.m_d01,m_d02+rhs.m_d02,
                      m_d10+rhs.m_d10,m_d11+rhs.m_d11,m_d12+rhs.m_d12,
                      m_d20+rhs.m_d20,m_d21+rhs.m_d21,m_d22+rhs.m_d22);
	}//end  operator

	inline CMatrix3x3 operator-(CMatrix3x3 rhs) const
	{
		return CMatrix3x3(m_d00-rhs.m_d00,m_d01-rhs.m_d01,m_d02-rhs.m_d02,
                      m_d10-rhs.m_d10,m_d11-rhs.m_d11,m_d12-rhs.m_d12,
                      m_d20-rhs.m_d20,m_d21-rhs.m_d21,m_d22-rhs.m_d22);
	}//end  operator

	inline CMatrix3x3 operator*(T num) const
	{
		// Return scaled matrix
		return CMatrix3x3(num*m_d00, num*m_d01, num*m_d02,
                      num*m_d10, num*m_d11, num*m_d12,
					            num*m_d20, num*m_d21, num*m_d22);
	}//end  operator

	inline const CMatrix3x3& operator+=(const CMatrix3x3 rhs) 
	{
		m_d00+=rhs.m_d00; m_d01+=rhs.m_d01; m_d02+=rhs.m_d02;
    m_d10+=rhs.m_d10; m_d11+=rhs.m_d11; m_d12+=rhs.m_d12;
    m_d20+=rhs.m_d20; m_d21+=rhs.m_d21; m_d22+=rhs.m_d22;
    return *this;
	}//end  operator
  
/**
*  Returns a matrix element, without checking for a valid user input, that means
*  if row or col are greater than the matrix dimension, we get a crash
*  @row The row index of the element
*  @col The column index of the element
*  @return Returns the element with index (row,col)
*/
	inline T& operator() (unsigned int row, unsigned int col)
	{
		return m_dEntries[3*row + col];
	}//end operator()
	
/**
*  Returns a matrix element, without checking for a valid user input, that means
*  if row or col are greater than the matrix dimension, we get a crash
*  @row The row index of the element
*  @col The column index of the element
*  @return Returns the element with index (row,col)
*/
	inline T operator() (unsigned int row, unsigned int col) const
	{
		return m_dEntries[3*row + col];
	}//end operator()
	
//===================================================
//  			Matrix Entry function
//===================================================
	inline T GetEntry(unsigned int row, unsigned int col) const
	{
		return m_dEntries[3*row + col];
	}//end operator()	

//===================================================
//  			output operator		
//===================================================
	template<typename Templateparm>
	friend std::ostream &operator << (std::ostream &out, const CMatrix3x3<Templateparm> &rhs);

};

template<class T>
std::ostream &operator << (std::ostream &out, const CMatrix3x3<T> &rhs)
{
	for(int i = 0; i < 3; i++)
	{
		for(int j = 0; j < 3; j++)
		{
			out<< rhs.m_dEntries[i*3+j]<<" ";
		}//end for
		out<<std::endl;
	}//end for
	return out;
}//end operator <<

typedef CMatrix3x3<double> CMatrix3x3d;
typedef CMatrix3x3<float>  CMatrix3x3f;
typedef CMatrix3x3<Real> MATRIX3X3;

#define IJ3(i,j) (i*3 + j)

}

#endif
