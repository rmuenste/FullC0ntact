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

#include "quaternion.h"

namespace i3d {


template <class T>   
void Quaternion<T>::CreateFromEulerAngles(T y0, T z0, T x0)
{
  T c1 = cos(y0/T(2.0));
  T s1 = sin(y0/T(2.0));
  T c2 = cos(z0/T(2.0));
  T s2 = sin(z0/T(2.0));
  T c3 = cos(x0/T(2.0));
  T s3 = sin(x0/T(2.0));
  
  T c1c2 = c1*c2;
  T s1s2 = s1*s2;
  w      = c1c2 * c3 - s1s2 * s3;
  x      = c1c2 * s3 + s1s2 * c3;
  y      = s1 * c2 * c3 + c1 * s2 * s3;
  z      = c1 * s2 * c3 - s1 * c2 * s3;
  
}
  
template <class T>  
Matrix3x3< T > Quaternion<T>::GetMatrix() const
{
  
  Matrix3x3<T> mat;
  
  T xx = x * x;
  T xy = x * y;
  T xz = x * z;
  T xw = x * w;
  T yy = y * y;
  T yz = y * z;
  T yw = y * w;
  T zz = z * z;
  T zw = z * w;

  mat.m_dEntries[0] = 1 - 2 * (yy + zz);
  mat.m_dEntries[1] = 2 * ( xy - zw );
  mat.m_dEntries[2] = 2 * ( xz + yw );

  mat.m_dEntries[3] = 2 * (xy + zw);
  mat.m_dEntries[4] = 1 - 2 * (xx + zz);
  mat.m_dEntries[5] = 2 * (yz - xw);

  mat.m_dEntries[6]  = 2 * (xz - yw);
  mat.m_dEntries[7]  = 2 * (yz + xw);
  mat.m_dEntries[8] = 1 - 2 * (xx + yy);
  
  return mat;

}
  
  
template <class T>
void Quaternion<T>::CreateMatrix(CMatrix4x4<T> &pMatrix)
{

	T xx = x * x;
	T xy = x * y;
	T xz = x * z;
	T xw = x * w;
	T yy = y * y;
	T yz = y * z;
	T yw = y * w;
	T zz = z * z;
	T zw = z * w;

	pMatrix.m_Entries[0] = 1 - 2 * (yy + zz);
	pMatrix.m_Entries[1] = 2 * ( xy + zw );
	pMatrix.m_Entries[2] = 2 * ( xz - yw );
	pMatrix.m_Entries[3] = 0;

	pMatrix.m_Entries[4] = 2 * (xy - zw);
	pMatrix.m_Entries[5] = 1 - 2 * (xx + zz);
	pMatrix.m_Entries[6] = 2 * (yz + xw);
	pMatrix.m_Entries[7] = 0;

	pMatrix.m_Entries[8]  = 2 * (xz + yw);
	pMatrix.m_Entries[9]  = 2 * (yz - xw);
	pMatrix.m_Entries[10] = 1 - 2 * (xx + yy);
	pMatrix.m_Entries[11] = 0;

	pMatrix.m_Entries[12] = 0;
	pMatrix.m_Entries[13] = 0;
	pMatrix.m_Entries[14] = 0;
	pMatrix.m_Entries[15] = 1;

}//end CreateMatrix

template <class T>
void Quaternion<T>::CreateMatrix(CMatrix4x4<T> &pMatrix) const
{

	T xx = x * x;
	T xy = x * y;
	T xz = x * z;
	T xw = x * w;
	T yy = y * y;
	T yz = y * z;
	T yw = y * w;
	T zz = z * z;
	T zw = z * w;

	pMatrix.m_Entries[0] = 1 - 2 * (yy + zz);
	pMatrix.m_Entries[1] = 2 * ( xy + zw );
	pMatrix.m_Entries[2] = 2 * ( xz - yw );
	pMatrix.m_Entries[3] = 0;

	pMatrix.m_Entries[4] = 2 * (xy - zw);
	pMatrix.m_Entries[5] = 1 - 2 * (xx + zz);
	pMatrix.m_Entries[6] = 2 * (yz + xw);
	pMatrix.m_Entries[7] = 0;

	pMatrix.m_Entries[8]  = 2 * (xz + yw);
	pMatrix.m_Entries[9]  = 2 * (yz - xw);
	pMatrix.m_Entries[10] = 1 - 2 * (xx + yy);
	pMatrix.m_Entries[11] = 0;

	pMatrix.m_Entries[12] = 0;
	pMatrix.m_Entries[13] = 0;
	pMatrix.m_Entries[14] = 0;
	pMatrix.m_Entries[15] = 1;

}//end CreateMatrix

template <class T>
void Quaternion<T>::AxisAngleToQuat(Vector3<T> vAxis, T W)
{

	T p = static_cast<T>(180);

	T angle = static_cast<T>(( W / p) * PI) ;

	T sine_angle = static_cast<T>(sin(angle / static_cast<T>(2) ));

	w = static_cast<T>(cos(angle / static_cast<T>(2) ));

	x = static_cast<T> (vAxis.x * sine_angle);
	y = static_cast<T> (vAxis.y * sine_angle);
	z = static_cast<T> (vAxis.z * sine_angle);

}//end AxisAngleToQuat

template <class T>
void Quaternion<T>::AxisAngleToQuat(T X, T Y, T Z, T W)
{

	T p = static_cast<T>(180);

	T angle = static_cast<T>(( W / p) * PI) ;

	T sine_angle = static_cast<T>(sin(angle / static_cast<T>(2) ));

	w = static_cast<T>(cos(angle / static_cast<T>(2) ));

	x = static_cast<T> (X * sine_angle);
	y = static_cast<T> (Y * sine_angle);
	z = static_cast<T> (Z * sine_angle);

}//end AxisAngleToQuat

//----------------------------------------------------------------------------
// Explicit instantiation.
//----------------------------------------------------------------------------
template class Quaternion<float>;

template class Quaternion<double>;
//----------------------------------------------------------------------------

}

