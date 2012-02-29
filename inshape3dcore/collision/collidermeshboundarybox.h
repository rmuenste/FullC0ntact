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
#ifndef COLLIDERMESHBOUNDARYBOX_H
#define COLLIDERMESHBOUNDARYBOX_H



//===================================================
//                     INCLUDES
//===================================================
#include<collider.h>

namespace i3d {

/**
 * @brief A collider for a mesh object and a box-shaped boundary
 *
 */  
  
class CColliderMeshBoundaryBox : public CCollider {

public: 

CColliderMeshBoundaryBox(); 

~CColliderMeshBoundaryBox(); 

void Collide(CRigidBody *pBody0, CRigidBody *pBody1, std::vector<CContact> &vContacts, Real dDeltaT);

void Collide(std::vector<CContact> &vContacts, Real dDeltaT);


};

}
#endif
