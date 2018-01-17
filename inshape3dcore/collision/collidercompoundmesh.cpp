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
#include "collidercompoundmesh.h"
#include <compoundbody.h>
#include <boundarybox.h>
#include "collisioninfo.h"
#include "rigidbody.h"
#include <meshobject.h>
#include <colliderfactory.h>
#include <sphere.h>
#include <distancepointrec.h>
#include <world.h>
#include <cylinder.h>
#include <quaternion.h>
#include <distancemeshsphere.h>

namespace i3d {

class ColliderMeshSphereInternal : public Collider
{
public:

  ColliderMeshSphereInternal(void){};

  ~ColliderMeshSphereInternal(void){};

 /**
 * @see CCollider::Collide
 *
 */
  void collide(std::vector<Contact> &vContacts)
  {
    //calculate the distance
    MeshObjectr *pMeshObjectOrig = dynamic_cast<MeshObjectr*>(body0_->shape_);
    Spherer *pSphere = dynamic_cast<Spherer *>(body1_->shape_);
    VECTOR3 trans = body1_->getTransformedPosition();
    Spherer sphere(trans,pSphere->getRadius());

    //distance to bounding box greater than eps
    CDistanceMeshSphere<Real> distMeshSphere(&pMeshObjectOrig->getBvhTree(),sphere);
    Real dist = distMeshSphere.ComputeDistanceEpsNaive( 0.1 * pSphere->getRadius());

    std::vector<VECTOR3>::iterator viter = distMeshSphere.m_vPoint.begin();
    int j=0;

    for(;viter!=distMeshSphere.m_vPoint.end();viter++,j++)
    {
      //std::cout<<"Distance to mesh: "<<dist+pSphere->getRadius()<<std::endl;
      VECTOR3 vPoint = *viter;
      //VECTOR3 position = sphere->Center() - (dist*0.5) * pPlane->m_vNormal;
      VECTOR3 vR0 = vPoint-body0_->com_;
      VECTOR3 vR1 = vPoint-body1_->com_;

      VECTOR3 relVel =
        (body0_->velocity_ + (VECTOR3::Cross(body0_->getAngVel(),vR0))
        - body1_->velocity_ - (VECTOR3::Cross(body1_->getAngVel(),vR1)));

      Real relativeNormalVelocity = relVel * distMeshSphere.m_vNormals[j];
      //if the bodies are on collision course
      if(relativeNormalVelocity < 0.0 && dist < 0.1 * pSphere->getRadius())
      {
        //std::cout<<"Pre-contact normal velocity: "<<relVel<<" colliding contact"<<std::endl;
        Contact contact;
        contact.m_dDistance  = dist+sphere.getRadius();
        contact.m_vNormal    = distMeshSphere.m_vNormals[j];
        contact.m_vPosition0 = vPoint;
        contact.m_vPosition1 = vPoint;
        contact.m_pBody0     = body0_;
        contact.m_pBody1     = body1_;
        contact.id0          = contact.m_pBody0->iID_;
        contact.id1          = contact.m_pBody1->iID_;
        contact.vn           = relativeNormalVelocity;
        contact.m_iState     = CollisionInfo::TOUCHING;
        vContacts.push_back(contact);
      }
      else if(dist < 0.1 * pSphere->getRadius())
      {
        //std::cout<<"Pre-contact normal velocity: "<<relVel<<" resting contact"<<std::endl;
        Contact contact;
        contact.m_dDistance  = dist+sphere.getRadius();
        contact.m_vNormal    = distMeshSphere.m_vNormals[j];
        contact.m_vPosition0 = vPoint;
        contact.m_vPosition1 = vPoint;
        contact.m_pBody0     = body0_;
        contact.m_pBody1     = body1_;
        contact.id0          = contact.m_pBody0->iID_;
        contact.id1          = contact.m_pBody1->iID_;
        contact.vn           = relativeNormalVelocity;
        contact.m_iState     = CollisionInfo::TOUCHING;
        vContacts.push_back(contact);
      }
    }//end for
  }

};

ColliderCompoundMesh::ColliderCompoundMesh()
{

}

ColliderCompoundMesh::~ColliderCompoundMesh()
{

}
	
void ColliderCompoundMesh::collide(std::vector<Contact> &vContacts)
{
	
	//loop over all bodies forming the compound
	CompoundBody *cbody0_ = dynamic_cast<CompoundBody*>(body0_);
	MeshObjectr *meshObject = dynamic_cast<MeshObjectr*>(body1_->shape_);
	for (int i=0; i<cbody0_->getNumComponents(); i++)
	{
		RigidBody *p0 = cbody0_->getComponent(i);
		//get shapeID of the body 
		if ( p0->getShape() == RigidBody::SPHERE)
		{
			//use collision detection for sphere
			//Check every pair
			ColliderFactory colliderFactory;

			//get a collider
			Collider *collider = new ColliderMeshSphereInternal();
			collider->setBody0(body1_);
			collider->setBody1(p0);

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
          contact.type1 = RigidBody::MESH;
				}
				vContacts.insert(vContacts.begin(), c.begin(), c.end());
			}
			delete collider;
		}

	}
		//for all other cases, add respective method here
}

}
