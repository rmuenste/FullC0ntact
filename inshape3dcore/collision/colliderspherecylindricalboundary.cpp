/*
    <one line to give the library's name and an idea of what it does.>
    Copyright (C) 2013  <copyright holder> <email>

    This library is free software; you can redistribute it and/or
    modify it under the terms of the GNU Lesser General Public
    License as published by the Free Software Foundation; either
    version 2.1 of the License, or (at your option) any later version.

    This library is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
    Lesser General Public License for more details.

    You should have received a copy of the GNU Lesser General Public
    License along with this library; if not, write to the Free Software
    Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
*/


#include "colliderspherecylindricalboundary.h"
#include <sphere.h>
#include <collisioninfo.h>
#include <distancepointrec.h>
#include <world.h>
#include <cylinder.h>
#include <quaternion.h>

namespace i3d {

CColliderSphereCylindricalBoundary::CColliderSphereCylindricalBoundary(void)
{
}

CColliderSphereCylindricalBoundary::~CColliderSphereCylindricalBoundary(void)
{

}

void CColliderSphereCylindricalBoundary::Collide(std::vector< CContact >& vContacts)
{

  CSpherer *sphere      = dynamic_cast<CSpherer *>(m_pBody0->m_pShape);
  CCylinderr *pBoundary = dynamic_cast<CCylinderr *>(m_pBody1->m_pShape);

  Real dist = 0.0;
  MATRIX3X3 local2World = m_pBody1->GetTransformationMatrix();
  MATRIX3X3 world2Local = local2World;
  world2Local.TransposeMatrix();

  // transform into the coordinate system of the cylinder
  VECTOR3 vLocal = m_pBody0->m_vCOM - m_pBody1->m_vCOM;
  vLocal = world2Local * vLocal;

  dist = sqrtf((vLocal.x*vLocal.x) + (vLocal.y*vLocal.y));
  dist = fabs(pBoundary->GetRadius()-(dist+sphere->Radius()));
  VECTOR3 vNormal = vLocal - VECTOR3(0,0,vLocal.z);
  vNormal.Normalize();
  VECTOR3 vContact = VECTOR3(0,0,vLocal.z) + dist * vNormal;
  vNormal = -vNormal;
  vNormal = local2World * vNormal;
  Real relVel = m_pBody0->m_vVelocity * vNormal;
  if(dist < 0.1*sphere->Radius())
  {
    CContact contact;
    contact.m_vNormal= vNormal;
    contact.m_vPosition0 = (local2World * vContact);
    contact.m_dDistance  = dist;
    contact.m_vPosition1 = contact.m_vPosition0;
    contact.m_pBody0     = m_pBody0;
    contact.m_pBody1     = m_pBody1;
    contact.id0 = contact.m_pBody0->m_iID;
    contact.id1 = contact.m_pBody1->m_iID;
    contact.m_iState     = CCollisionInfo::TOUCHING;
    vContacts.push_back(contact);
  }

}


}
