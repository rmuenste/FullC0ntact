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

template<class T>
const T CMath<T>::SYS_PI(3.14159265358979323846);

template<class T>
const T CMath<T>::EPSILON7(1.0e-8);

template<class T>
const T CMath<T>::EPSILON5(1.0e-6);

template<class T>
const T CMath<T>::EPSILON4(1.0e-5);

template<class T>
const T CMath<T>::EPSILON3(1.0e-4);

typedef double Real;

}

#endif

