/*
    <one line to give the program's name and a brief idea of what it does.>
    Copyright (C) <year>  <name of author>

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

#include "compoundbody.h"
#include <sphere.h>
#include <boundarybox.h>
#include <rigidbodyio.h>
#include <stdlib.h>
#include <cylinder.h>
#include <meshobject.h>
#include <genericloader.h>
#include <3dmodel.h>
#include <subdivisioncreator.h>
#include <collisioninfo.h>

namespace i3d {

	CompoundBody::CompoundBody() : RigidBody()
	{

	}

	CompoundBody::~CompoundBody()
	{
		std::vector<RigidBody*>::iterator i = rigidBodies_.begin();
		for (; i != rigidBodies_.end(); i++)
		{
			RigidBody *body = *i;
			delete body;
		}
	}

	//constructor from  body storage object 
	CompoundBody::CompoundBody(BodyStorage *pBody) :RigidBody(pBody){
		rigidBodies_ = pBody->rigidBodies_;
		numofComps_ = pBody->numofComps_;
	}

	CompoundBody::CompoundBody(const i3d::CompoundBody& copy) : RigidBody(copy)
	{
		//zusätzliches Attribut von compounds setzen
		rigidBodies_ = copy.rigidBodies_;
		numofComps_ = copy.numofComps_;
	}




	void CompoundBody::translateTo(const VECTOR3& vPos)
	{
		/**to translate all the bodies forming the compound, use this formula:
		   the vector pointing from the old com of the compound com_old to the com of body i com_i is given by com_i-com_old
		   thus, new com of body i com_inew is given by com_new + (com_i - com_old), where com_new represents the new com of the compound*/

		std::vector<RigidBody*>::iterator i = rigidBodies_.begin();
		for (; i != rigidBodies_.end(); i++)
		{
			//calculate new position of body i
			VECTOR3 pos_i = (*i)->com_;
			VECTOR3 newPos = vPos + pos_i - com_;

			//now translte body i
			(*i)->translateTo(newPos);
		}

		i3d::RigidBody::translateTo(vPos); //translates the com of the compound
	}

	void CompoundBody::generateInvInertiaTensor()
	{

		//for a compound body, the inertia tensor is obtained by adding the inertia tensors of its components
		//initiate as zero   
		
		MATRIX3X3 InertiaTensor = MATRIX3X3();
		InertiaTensor.SetZero();

		//now loop over all components, adding them up
		std::vector<RigidBody*>::iterator i = rigidBodies_.begin();
		for (; i != rigidBodies_.end(); i++)
		{
			InertiaTensor += (*i)->invInertiaTensor_;
		}
		invInertiaTensor_ = InertiaTensor;
		
	}

	//angular velocity is the same for all components. thus, only one value (the angVel of the compound) needs to be updated if necessary

    //anything containing a translational velocity update needs to be redefined, due to compound structure
	void CompoundBody::applyImpulse(const VECTOR3 &relPos, const VECTOR3 &impulse, const VECTOR3 &linearUpdate)
	{
		MATRIX3X3 mInvInertiaTensor = getWorldTransformedInvTensor();

		//update velocity of the compound
		velocity_ += linearUpdate;

		//the velocity of the components changes as well
		std::vector<RigidBody*>::iterator i = rigidBodies_.begin();
		for (; i != rigidBodies_.end(); i++)
		{

			//translational velocity of component i is given by 
			// velocity_i =  velocity_ + Cross((com_i -com_), angVel_)
			VECTOR3  &crossprod = VECTOR3::Cross(((*i)->com_ - com_), (*i)->getAngVel());
			(*i)->velocity_ = (*i)->velocity_ + crossprod;

		}



		//angVel doesn't need to be changed
		setAngVel(getAngVel() + mInvInertiaTensor * (VECTOR3::Cross(relPos, impulse)));
		 
	}

	//same for bias Impulse

	void CompoundBody::applyBiasImpulse(const VECTOR3 &relPos, const VECTOR3 &impulse, const VECTOR3 &linearUpdate)
	{

		MATRIX3X3 mInvInertiaTensor = getWorldTransformedInvTensor();
		//update velocity of compound
		biasVelocity_ += linearUpdate;

		//update component velocity
		std::vector<RigidBody*>::iterator i = rigidBodies_.begin();
		for (; i != rigidBodies_.end(); i++)
		{
			VECTOR3  &crossprod = VECTOR3::Cross(((*i)->com_ - com_), (*i)->getAngVel());
			(*i)->biasVelocity_ = (*i)->biasVelocity_ + crossprod;

		}

		setAngVel(getAngVel() + mInvInertiaTensor * (VECTOR3::Cross(relPos, impulse)));

	}
}