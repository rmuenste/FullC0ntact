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
#ifndef CONTACTGENERATOR2CYLINDER_H
#define CONTACTGENERATOR2CYLINDER_H



//===================================================
//                     INCLUDES
//===================================================
#include <contactgenerator.h>

namespace i3d {

/**
 * @brief A contact generator for two cylinders
 *
 */  
  
template<class T>
class CContactGenerator2Cylinder : public CContactGenerator<T> {

public: 

CContactGenerator2Cylinder(); 

~CContactGenerator2Cylinder(); 

void GenerateContactPoints(const Shape<T> &shape0, const Shape<T> &shape1, CSimplexDescriptorGjk<T> &simplex,
                           const CTransform<T> &transform0, const CTransform<T> &transform1,
                           const CVector3<T> &closestPoint0, const CVector3<T> &closestPoint1,
                           CVector3<T> &normal, int &nContacts, std::vector<CVector3<T> > &vContacts);

};

}
#endif
