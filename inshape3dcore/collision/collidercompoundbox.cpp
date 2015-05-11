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


//===================================================
//                     INCLUDES
//===================================================
#include "collidercompoundbox.h"
#include <compoundbody.h>
#include <boundarybox.h>
#include <collisioninfo.h>
#include <rigidbody.h>

#include <colliderfactory.h>
#include <sphere.h>
#include <distancepointrec.h>
#include <world.h>
#include <cylinder.h>
#include <quaternion.h>
#include <distancepointobb3.h>

namespace i3d {

class ColliderCompoundBoxInternal : public Collider
{
public:

  ColliderCompoundBoxInternal(void){};

  ~ColliderCompoundBoxInternal(void){};

  /**
  * @see CCollider::Collide
  *
  */
  void collide(std::vector<Contact> &vContacts)
  {
    Spherer *pSphere = dynamic_cast<Spherer *>(body0_->shape_);
    OBB3r *pBox = dynamic_cast<OBB3r *>(body1_->shape_);
    VECTOR3 trans = body0_->getTransformedPosition();
    Spherer sphere(trans, pSphere->getRadius());

    Real rad0 = sphere.getRadius();
    Real rad1 = pBox->getBoundingSphereRadius();
    const OBB3r &origBox = dynamic_cast<const OBB3r& >(body1_->getOriginalShape());

    VECTOR3 newPos0 = sphere.center_;

    //in the next time step
    if ((newPos0 - body1_->com_).mag() > rad0 + rad1)
      return;

    Transformationr newTransform(body1_->getTransformation().getMatrix(), body1_->com_);

    CDistancePointObb3<Real> distPointBox(*pBox, newPos0, newTransform);
    Real minDist = distPointBox.ComputeDistance();
    Real penetrationDepth = minDist - rad0;

    //Normal points from box to sphere
    if (penetrationDepth < 0.1*rad0)
    {
      VECTOR3 angPart = (VECTOR3::Cross(body1_->getAngVel(), distPointBox.m_vClosestPoint1 - body1_->com_));
      VECTOR3 relativeVelocity = body0_->velocity_ - body1_->velocity_ - angPart;

      //relative velocity along the normal
      Real normalVelocity = relativeVelocity * distPointBox.m_ocConf.m_vNormal;

      Contact contact;
      contact.m_dDistance = minDist;
      contact.m_vNormal = distPointBox.m_ocConf.m_vNormal;
      contact.m_vPosition0 = distPointBox.m_vClosestPoint1;
      contact.m_vPosition1 = distPointBox.m_vClosestPoint1;
      contact.m_pBody0 = body0_;
      contact.m_pBody1 = body1_;
      contact.id0 = contact.m_pBody0->iID_;
      contact.id1 = contact.m_pBody1->iID_;
      contact.vn = normalVelocity;
      contact.m_iState = CollisionInfo::TOUCHING;
      vContacts.push_back(contact);
    }

  }

};

ColliderCompoundBox::ColliderCompoundBox()
{

}

ColliderCompoundBox::~ColliderCompoundBox()
{

}
	

void ColliderCompoundBox::collide(std::vector<Contact> &vContacts)
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
      Collider *collider = new ColliderCompoundBoxInternal();
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
          contact.type1 = RigidBody::BOX;
        }
        vContacts.insert(vContacts.begin(), c.begin(), c.end());
      }
      delete collider;
    }

  }

}

}
