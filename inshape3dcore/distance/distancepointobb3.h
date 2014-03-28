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
#ifndef DISTANCEPOINTOBB3_H
#define DISTANCEPOINTOBB3_H



//===================================================
//                     INCLUDES
//===================================================
#include <distance.h>
#include <obb3.h>
#include <transform.h>
#include <distancetools.h>

namespace i3d {

/**
* @brief Computes the distance between a point and a box in 3d
*
*
*/
template <class T>
class CDistancePointObb3 : public CDistance<T> {

public: 

CDistancePointObb3(const OBB3<T> &box, const CVector3<T> &point, const Transformation<T> &tranform) : m_pBox(&box), m_pPoint(point),
                   m_pTransform(&tranform) {}; 

~CDistancePointObb3(); 

  T ComputeDistanceSqr();
  T ComputeDistance();

  const OBB3<T> *m_pBox;

  OBB3<T> m_pBoxTrans;
  
  CVector3<T> m_pPoint;
  
  const Transformation<T> *m_pTransform;
  
  CObjConfiguration<T> m_ocConf;
  
  using CDistance<T>::m_vClosestPoint0;
  using CDistance<T>::m_vClosestPoint1;  


};

}

#endif
