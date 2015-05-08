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


#include "collidercompcylindricalboundary.h"
#include <sphere.h>
#include <collisioninfo.h>
#include <distancepointrec.h>
#include <world.h>
#include <cylinder.h>
#include <quaternion.h>
#include <boundarycyl.h>

namespace i3d {

class ColliderCompCylInternal : public Collider
{
public:

  ColliderCompCylInternal(void){};

  ~ColliderCompCylInternal(void){};

  /**
  * @see CCollider::Collide
  *
  */
  void collide(std::vector<Contact> &vContacts)
  {
    if(body1_->shapeId_ == RigidBody::HOLLOWCYLINDER)
    {
      collideHollow(vContacts);
      return;
    }

    Spherer *pSphere      = dynamic_cast<Spherer *>(body0_->shape_);
    BoundaryCylr *pBoundary = dynamic_cast<BoundaryCylr *>(body1_->shape_);
    VECTOR3 trans = body0_->getTransformedPosition();
    Spherer sphere(trans,pSphere->getRadius());

    Real dist = 0.0;
    MATRIX3X3 local2World = body1_->getTransformationMatrix();
    MATRIX3X3 world2Local = local2World;
    world2Local.TransposeMatrix();

    // transform into the coordinate system of the cylinder
    VECTOR3 vLocal = trans - body1_->com_;
    vLocal = world2Local * vLocal;

    dist = sqrtf((vLocal.x*vLocal.x) + (vLocal.y*vLocal.y));
    dist = fabs(pBoundary->cylinder_.getRadius()-(dist+sphere.getRadius()));
    VECTOR3 vNormal = vLocal - VECTOR3(0,0,vLocal.z);
    vNormal.Normalize();
    VECTOR3 vContact = VECTOR3(0,0,vLocal.z) + dist * vNormal;
    vNormal = -vNormal;
    vNormal = local2World * vNormal;
    Real relVel = body0_->velocity_ * vNormal;
    //distance to side of cylinder
    if(dist < 0.1*sphere.getRadius())
    {
      Contact contact;
      contact.m_vNormal= vNormal;
      contact.m_vPosition0 = (local2World * vContact) + body1_->com_;
      contact.m_dDistance  = fabs(dist);
      contact.m_vPosition1 = contact.m_vPosition0;
      contact.m_pBody0     = body0_;
      contact.m_pBody1     = body1_;
      contact.id0 = contact.m_pBody0->iID_;
      contact.id1 = contact.m_pBody1->iID_;
      contact.m_iState     = CollisionInfo::TOUCHING;
      vContacts.push_back(contact);
    }

    //distance to bottom
    dist = fabs(fabs(-pBoundary->cylinder_.getHalfLength() - vLocal.z) - sphere.getRadius());

    vNormal = VECTOR3(0,0,1);

    vContact = VECTOR3(vLocal.x,vLocal.y,vLocal.z - sphere.getRadius());
    vNormal = local2World * vNormal;
    if(dist < 0.1*sphere.getRadius())
    {
      Contact contact;
      contact.m_vNormal= vNormal;
      contact.m_vPosition0 = (local2World * vContact) + body1_->com_;
      contact.m_dDistance  = fabs(dist)+pSphere->getRadius();
      contact.m_vPosition1 = contact.m_vPosition0;
      contact.m_pBody0     = body0_;
      contact.m_pBody1     = body1_;
      contact.id0 = contact.m_pBody0->iID_;
      contact.id1 = contact.m_pBody1->iID_;
      contact.m_iState     = CollisionInfo::TOUCHING;
      vContacts.push_back(contact);
    }

    //distance to top
    dist = fabs(fabs(pBoundary->cylinder_.getHalfLength() - vLocal.z) - sphere.getRadius());

    vNormal = VECTOR3(0,0,-1);

    vContact = VECTOR3(vLocal.x,vLocal.y,vLocal.z + sphere.getRadius());
    vNormal = local2World * vNormal;
    if(dist < 0.1*sphere.getRadius())
    {
      Contact contact;
      contact.m_vNormal= vNormal;
      contact.m_vPosition0 = (local2World * vContact) + body1_->com_;
      contact.m_dDistance  = fabs(dist)+pSphere->getRadius();
      contact.m_vPosition1 = contact.m_vPosition0;
      contact.m_pBody0     = body0_;
      contact.m_pBody1     = body1_;
      contact.id0 = contact.m_pBody0->iID_;
      contact.id1 = contact.m_pBody1->iID_;
      contact.m_iState     = CollisionInfo::TOUCHING;
      vContacts.push_back(contact);
    }

  }

  void collideHollow(std::vector< Contact >& vContacts)
  {

    Spherer *pSphere      = dynamic_cast<Spherer *>(body0_->shape_);
    BoundaryCylr *pBoundary = dynamic_cast<BoundaryCylr *>(body1_->shape_);

    VECTOR3 trans = body0_->getTransformedPosition();
    Spherer sphere(trans,pSphere->getRadius());

    Real dist = 0.0;
    MATRIX3X3 local2World = body1_->getTransformationMatrix();
    MATRIX3X3 world2Local = local2World;
    world2Local.TransposeMatrix();

    // transform into the coordinate system of the cylinder
    VECTOR3 vLocal = trans - body1_->com_;
    vLocal = world2Local * vLocal;

    dist = sqrtf((vLocal.x*vLocal.x) + (vLocal.y*vLocal.y));
    dist = fabs(pBoundary->cylinder_.getRadius()-(dist+sphere.getRadius()));
    VECTOR3 vNormal = vLocal - VECTOR3(0,0,vLocal.z);
    vNormal.Normalize();
    VECTOR3 vContact = VECTOR3(0,0,vLocal.z) + dist * vNormal;
    vNormal = -vNormal;
    vNormal = local2World * vNormal;
    Real relVel = body0_->velocity_ * vNormal;

    //distance to side of cylinder
    if(dist < 0.1*sphere.getRadius())
    {
      Contact contact;
      contact.m_vNormal= vNormal;
      contact.m_vPosition0 = (local2World * vContact) + body1_->com_;
      contact.m_dDistance  = fabs(dist);
      contact.m_vPosition1 = contact.m_vPosition0;
      contact.m_pBody0     = body0_;
      contact.m_pBody1     = body1_;
      contact.id0 = contact.m_pBody0->iID_;
      contact.id1 = contact.m_pBody1->iID_;
      contact.m_iState     = CollisionInfo::TOUCHING;
      vContacts.push_back(contact);
    }

    dist = sqrtf((vLocal.x*vLocal.x) + (vLocal.y*vLocal.y));
    dist = dist - (sphere.getRadius()+0.015);
    //dist = fabs(pBoundary->cylinder_.getRadius()-(dist+sphere->getRadius()));
    vNormal = vLocal - VECTOR3(0,0,vLocal.z);
    vNormal.Normalize();
    vContact = VECTOR3(0,0,vLocal.z) + fabs(dist) * vNormal;
    vNormal = local2World * vNormal;
    relVel = body0_->velocity_ * vNormal;

    //distance to inner cylinder
    if(dist < 0.1*sphere.getRadius())
    {
      Contact contact;
      contact.m_vNormal= vNormal;
      contact.m_vPosition0 = (local2World * vContact) + body1_->com_;
      contact.m_dDistance  = fabs(dist)+pSphere->getRadius();
      contact.m_vPosition1 = contact.m_vPosition0;
      contact.m_pBody0     = body0_;
      contact.m_pBody1     = body1_;
      contact.id0 = contact.m_pBody0->iID_;
      contact.id1 = contact.m_pBody1->iID_;
      contact.m_iState     = CollisionInfo::TOUCHING;
      vContacts.push_back(contact);
    }

    //distance to bottom
    dist = fabs(fabs(-pBoundary->cylinder_.getHalfLength() - vLocal.z) - sphere.getRadius());

    vNormal = VECTOR3(0,0,1);

    vContact = VECTOR3(vLocal.x,vLocal.y,vLocal.z - sphere.getRadius());
    vNormal = local2World * vNormal;

    Real distcenter = fabs(-pBoundary->cylinder_.getHalfLength() - vLocal.z);
    //Real overlap = std::max(sphere->getRadius() - distcenter,0.0);

    if(dist < 0.1*sphere.getRadius())
    {
      Contact contact;
      contact.m_vNormal= vNormal;
      contact.m_vPosition0 = (local2World * vContact) + body1_->com_;
      contact.m_dDistance  = distcenter;
      contact.m_vPosition1 = contact.m_vPosition0;
      contact.m_pBody0     = body0_;
      contact.m_pBody1     = body1_;
      contact.id0 = contact.m_pBody0->iID_;
      contact.id1 = contact.m_pBody1->iID_;
      contact.m_iState     = CollisionInfo::TOUCHING;
      vContacts.push_back(contact);
    }

    //distance to top
    dist = fabs(fabs(pBoundary->cylinder_.getHalfLength() - vLocal.z) - sphere.getRadius());

    vNormal = VECTOR3(0,0,-1);

    vContact = VECTOR3(vLocal.x,vLocal.y,vLocal.z + sphere.getRadius());
    vNormal = local2World * vNormal;
    if(dist < 0.1*sphere.getRadius())
    {
      Contact contact;
      contact.m_vNormal= vNormal;
      contact.m_vPosition0 = (local2World * vContact) + body1_->com_;
      contact.m_dDistance  = fabs(dist)+pSphere->getRadius();
      contact.m_vPosition1 = contact.m_vPosition0;
      contact.m_pBody0     = body0_;
      contact.m_pBody1     = body1_;
      contact.id0 = contact.m_pBody0->iID_;
      contact.id1 = contact.m_pBody1->iID_;
      contact.m_iState     = CollisionInfo::TOUCHING;
      vContacts.push_back(contact);
    }

  }

};


ColliderCompCylindricalBoundary::ColliderCompCylindricalBoundary(void)
{
}

ColliderCompCylindricalBoundary::~ColliderCompCylindricalBoundary(void)
{

}

void ColliderCompCylindricalBoundary::collide(std::vector< Contact >& vContacts)
{

  //loop over all bodies forming the compound
  CompoundBody *cbody0_ = dynamic_cast<CompoundBody*>(body0_);

  for (int i = 0; i<cbody0_->getNumComponents(); i++)
  {
    RigidBody *p0 = cbody0_->getComponent(i);
    //get shapeID of the body
    if (p0->getShape() == RigidBody::SPHERE)
    {
      //use collision detection for sphere
      //Check every pair

      //get a collider
      Collider *collider = new ColliderCompCylInternal();
      collider->setBody0(p0);
      collider->setBody1(body1_);

      //attach the world object
      collider->setWorld(world_);
      std::vector<Contact> c;

      //compute the potential contact points; collide called from  component-component collision
      cbody0_->transform_.setOrigin(cbody0_->com_);
      MATRIX3X3 rot = cbody0_->getTransformationMatrix();
      cbody0_->transform_.setMatrix(rot);

      collider->collide(c);
      if (!c.empty())
      {
        for (auto &contact : c)
        {
          contact.subId1 = i;
          contact.cbody0 = cbody0_;
          contact.subId0 = i;
          contact.cbody1 = NULL;
          contact.type0 = RigidBody::COMPOUND;
          contact.type1 = RigidBody::HOLLOWCYLINDER;
        }
        vContacts.insert(vContacts.begin(), c.begin(), c.end());
      }
      delete collider;
    }

  }

}

}
