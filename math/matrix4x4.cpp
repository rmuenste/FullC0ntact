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

#include "matrix4x4.h"

namespace i3d {

std::ostream &operator << (std::ostream &out, const CMatrix4x4f &rhs)
{
	for(int i = 0; i < 4; i++)
	{
		for(int j = 0; j < 4; j++)
		{
			out<< rhs.m_Entries[i*4+j]<<" ";
		}//end for
		out<<std::endl;
	}//end for
	return out;
}//end operator <<

//===================================================
//			  construct via array
//===================================================
template <class T>
CMatrix4x4<T>::CMatrix4x4(T nEntries[])
{
	memcpy(m_Entries, nEntries, 16 * sizeof(T));
}//End constructor

//===================================================
//			Matrix vector product
//===================================================

template <class T>
Vector4<T> CMatrix4x4<T>::operator *(const Vector4<T> &vVec) const
{

	Vector4<T> res(0,0,0,0);
	
	for(int i = 0; i < 4; i++)
		for(int j = 0; j < 4; j++)
			res.m_dCoords[i] += m_Entries[i*4+j] * vVec.m_dCoords[j];

	return res;

}//end operator

//===================================================
//			     SubMatrix3x3
//===================================================
template<class T>
Matrix3x3<T> CMatrix4x4<T>::GetSubMatrix3x3(int i, int j) const
{

	T entries[9];

	int iTarget = -1;
  int jTarget = -1;

	for(int k = 0; k < 4; k++)
	{

		if(k < i)
		{
			iTarget = k;
		}
		else if(k > i)
		{
			iTarget = k-1;
		}

		for(int l = 0; l < 4; l++)
		{

			if(l < j)
			{
				jTarget = l;
			}
			else if(l > j)
			{
				jTarget = l-1;
			}
			
			if(k != i && l != j && iTarget >= 0 && jTarget >= 0)
			{
				entries[iTarget*3 + jTarget] = m_Entries[k*4+l];
			}

		}//end for l

	}//end for k

	return Matrix3x3<T>(entries);
}//end GetSubMatrix3x3

//===================================================
//  			Determinate		
//===================================================
template<class T>
T CMatrix4x4<T>::Determinate() const
{
	
	T det, res = 0;
	T i = 1;
	for(int n = 0; n < 4; n++, i*=-1)
	{
		Matrix3x3<T> mat = GetSubMatrix3x3(0,n);

		det = mat.Determinate();

		res+=m_Entries[n] * i * det;

	}//end for

	return res;

}//end Determinate

template<class T>
bool CMatrix4x4<T>::GetInverseMatrix(CMatrix4x4 &matInverse) const
{

	T nDeterminate = Determinate();

	int i, j, sign;

	if(fabs(nDeterminate) < 0.0005)
	{
		return false;
	}

	for(i = 0; i < 4; i++)
	{
		for(j = 0; j < 4; j++)
		{

			sign = 1 - ((i+j)%2) * 2;

			Matrix3x3<T> matSub = GetSubMatrix3x3(i,j);

			T det = matSub.Determinate();

			matInverse.m_Entries[i+j*4] = (det * sign)/nDeterminate;

		}//end for
	}//end for

	return true;
}//end GetInverseMatrix

//----------------------------------------------------------------------------
// Explicit instantiation.
//----------------------------------------------------------------------------
template class CMatrix4x4<float>;

template class CMatrix4x4<double>;
//----------------------------------------------------------------------------

}
