/*
    <one line to give the program's name and a brief idea of what it does.>
    Copyright (C) <2011>  <Raphael Muenster>

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
#ifndef CONE_H
#define CONE_H



//===================================================
//                     INCLUDES
//===================================================
#include <vector3.h>
#include <shape.h>
#include <convexshape.h>

namespace i3d {

/** @brief A class for a cone
 *
 * A class for a cone that is derived from the convexshape class
 * and implements the convexshape interface
 */      
template<class T>
class CCone : public CConvexShape<T> {

public: 

CCone(); 

~CCone(); 

CCone(const CVector3<T> &center, const CVector3<T> u, T radius, T h2) : m_vCenter(center),
          m_vU(u), m_dRadius(radius), m_dHalfLength(h2) {};

CCone(const CCone<T> &copy)
{
  m_vCenter     = copy.m_vCenter;
  m_vU          = copy.m_vU;
  m_dRadius     = copy.m_dRadius;
  m_dHalfLength = copy.m_dHalfLength;
}

CVector3<T> GetSupport(const CVector3<T> &v) const;

CVector3<T> GetPointOnBoundary() const
{
  //return apex
  return m_vCenter + m_dHalfLength * m_vU;
};

CAABB3<T> GetAABB(){ T extents[3]={m_dRadius,m_dRadius,m_dHalfLength};return CAABB3<T>(m_vCenter,extents);};

T Volume() const {return CMath<T>::SYS_PI * m_dRadius * m_dRadius * (2.0 * m_dHalfLength);};


inline CVector3<T> eval(T phi) const
{
  return CVector3<T>(m_vCenter.x+m_dRadius*cos(phi), m_vCenter.y+m_dRadius*sin(phi),0);
}


inline CVector3<T> GetCenter() const {return m_vCenter;};
inline CVector3<T> GetU() const {return m_vU;};
inline T GetRadius() const {return m_dRadius;};
inline T GetHalfLength() const {return m_dHalfLength;};
inline void SetCenter(const CVector3<T> &center) {m_vCenter=center;};
inline void SetU(const CVector3<T> &u) {m_vU=u;};
inline void SetRadius(T radius) {m_dRadius=radius;};
inline void SetHalfLength(T halfLength) {m_dHalfLength=halfLength;};

private:

  CVector3<T> m_vCenter;
  CVector3<T> m_vU;
  T                     m_dRadius;
  T                     m_dHalfLength;

};

typedef CCone<float> CConef;
typedef CCone<double> CConed;
typedef CCone<Real> CConer;


}
#endif

