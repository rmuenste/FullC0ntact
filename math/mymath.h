/***************************************************************************
 *   Copyright (C) 2009 by Raphael M�nster   *
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

#ifndef CMATH_H_
#define CMATH_H_

#include<limits>
#include<cmath>

#ifdef __CUDACC__
#define host_dev __device__ __host__
#else
#define host_dev 
#endif

namespace i3d {

/**
* @brief A class for math declarations, constants and functions
*
*
*/
template <class T>
class CMath
{
public:

  static T Sqrt(T dVal);
  static T InvSqrt(T dVal);

  static const T EPS1;
  static const T MAXREAL;
  static const T MINREAL;
  static const T SYS_PI;

  static const T EPSILON7;
  static const T EPSILON5;
  static const T EPSILON4;
  static const T EPSILON3;
  static const T EPSILON1;

  static const T TOLERANCEZERO;

};

//template<class T>
//const T CMath<T>::EPS1((T)1e-04);

template<class T>
const T CMath<T>::MAXREAL=std::numeric_limits<T>::max();

template<class T>
const T CMath<T>::MINREAL=std::numeric_limits<T>::min();


typedef double Real;



}

#endif

