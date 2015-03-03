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
#include "collidercompoundboundarybox.h"
#include <compoundbody.h>
#include <boundarybox.h>
#include "collisioninfo.h"
#include "rigidbody.h"

#include <colliderfactory.h>
#include <sphere.h>
#include <distancepointrec.h>
#include <world.h>
#include <cylinder.h>
#include <quaternion.h>

namespace i3d {


ColliderCompoundBoundaryBox::ColliderCompoundBoundaryBox()
{

}

ColliderCompoundBoundaryBox::~ColliderCompoundBoundaryBox()
{

}
	

void ColliderCompoundBoundaryBox::collide(std::vector<Contact> &vContacts)
{
	
	//loop over all bodies forming the compound
	CompoundBody *cbody0_ = dynamic_cast<CompoundBody*>(body0_);
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
			Collider *collider = colliderFactory.ProduceCollider(p0, body1_);

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
					contact.subId0 = i;
					contact.cbody0 = cbody0_;
          contact.type0 = RigidBody::COMPOUND;
          contact.type1 = RigidBody::BOUNDARYBOX;
				}
				vContacts.insert(vContacts.begin(), c.begin(), c.end());
			}
			delete collider;
		}

	}
		//for all other cases, add respective method here
}

}
