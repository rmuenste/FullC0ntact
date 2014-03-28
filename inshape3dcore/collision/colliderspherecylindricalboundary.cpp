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
#include <boundarycyl.h>

namespace i3d {

CColliderSphereCylindricalBoundary::CColliderSphereCylindricalBoundary(void)
{
}

CColliderSphereCylindricalBoundary::~CColliderSphereCylindricalBoundary(void)
{

}

void CColliderSphereCylindricalBoundary::Collide(std::vector< Contact >& vContacts)
{

  CSpherer *sphere      = dynamic_cast<CSpherer *>(m_pBody0->shape_);
  CBoundaryCylr *pBoundary = dynamic_cast<CBoundaryCylr *>(m_pBody1->shape_);

  Real dist = 0.0;
  MATRIX3X3 local2World = m_pBody1->getTransformationMatrix();
  MATRIX3X3 world2Local = local2World;
  world2Local.TransposeMatrix();

  // transform into the coordinate system of the cylinder
  VECTOR3 vLocal = m_pBody0->com_ - m_pBody1->com_;
  vLocal = world2Local * vLocal;
  
  dist = sqrtf((vLocal.x*vLocal.x) + (vLocal.y*vLocal.y));
  dist = fabs(pBoundary->m_Cylinder.GetRadius()-(dist+sphere->Radius()));
  VECTOR3 vNormal = vLocal - VECTOR3(0,0,vLocal.z);
  vNormal.Normalize();
  VECTOR3 vContact = VECTOR3(0,0,vLocal.z) + dist * vNormal;
  vNormal = -vNormal;
  vNormal = local2World * vNormal;
  Real relVel = m_pBody0->velocity_ * vNormal;
  //distance to side of cylinder
  if(dist < 0.1*sphere->Radius())
  {
    Contact contact;
    contact.m_vNormal= vNormal;
    contact.m_vPosition0 = (local2World * vContact) + m_pBody1->com_;
    contact.m_dDistance  = dist;
    contact.m_vPosition1 = contact.m_vPosition0;
    contact.m_pBody0     = m_pBody0;
    contact.m_pBody1     = m_pBody1;
    contact.id0 = contact.m_pBody0->iID_;
    contact.id1 = contact.m_pBody1->iID_;
    contact.m_iState     = CollisionInfo::TOUCHING;
    vContacts.push_back(contact);
  }

  //distance to bottom
  dist = fabs(fabs(-pBoundary->m_Cylinder.GetHalfLength() - vLocal.z) - sphere->Radius());

  vNormal = VECTOR3(0,0,1);

  vContact = VECTOR3(vLocal.x,vLocal.y,vLocal.z - sphere->Radius());
  vNormal = local2World * vNormal;
  if(dist < 0.1*sphere->Radius())
  {
    Contact contact;
    contact.m_vNormal= vNormal;
    contact.m_vPosition0 = (local2World * vContact) + m_pBody1->com_;
    contact.m_dDistance  = dist;
    contact.m_vPosition1 = contact.m_vPosition0;
    contact.m_pBody0     = m_pBody0;
    contact.m_pBody1     = m_pBody1;
    contact.id0 = contact.m_pBody0->iID_;
    contact.id1 = contact.m_pBody1->iID_;
    contact.m_iState     = CollisionInfo::TOUCHING;
    vContacts.push_back(contact);
  }
  
  //distance to top
  dist = fabs(fabs(pBoundary->m_Cylinder.GetHalfLength() - vLocal.z) - sphere->Radius());

  vNormal = VECTOR3(0,0,-1);

  vContact = VECTOR3(vLocal.x,vLocal.y,vLocal.z + sphere->Radius());
  vNormal = local2World * vNormal;
  if(dist < 0.1*sphere->Radius())
  {
    Contact contact;
    contact.m_vNormal= vNormal;
    contact.m_vPosition0 = (local2World * vContact) + m_pBody1->com_;
    contact.m_dDistance  = dist;
    contact.m_vPosition1 = contact.m_vPosition0;
    contact.m_pBody0     = m_pBody0;
    contact.m_pBody1     = m_pBody1;
    contact.id0 = contact.m_pBody0->iID_;
    contact.id1 = contact.m_pBody1->iID_;
    contact.m_iState     = CollisionInfo::TOUCHING;
    vContacts.push_back(contact);
  }  
  

}


}
