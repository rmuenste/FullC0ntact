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
#define E1 0.1   
#define E3 0.0001   
#define E4 0.00001  
#define E5 0.000001 

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

  static const T TOLERANCEZERO;

};

//template<class T>
//const T CMath<T>::EPS1((T)1e-04);

template<class T>
const T CMath<T>::MAXREAL=std::numeric_limits<T>::max();

template<class T>
const T CMath<T>::MINREAL=std::numeric_limits<T>::min();

typedef double Real;

template<>
const Real CMath<Real>::SYS_PI(3.14159265358979323846);

template<>
const Real CMath<Real>::EPSILON7(1.0e-8);

template<>
const Real CMath<Real>::EPSILON5(1.0e-6);

template<>
const Real CMath<Real>::EPSILON4(1.0e-5);

template<>
const Real CMath<Real>::EPSILON3(1.0e-4);

template<>
const float CMath<float>::SYS_PI(3.14159265358979323846f);

template<>
const float CMath<float>::EPSILON7(1.0e-8f);

template<>
const float CMath<float>::EPSILON5(1.0e-6f);

template<>
const float CMath<float>::EPSILON4(1.0e-5f);

template<>
const float CMath<float>::EPSILON3(1.0e-4f);


}

#endif

