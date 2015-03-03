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
  shapeId_ = RigidBody::COMPOUND;
	this->affectedByGravity_ = true;
	this->friction_ = 0.0;
	this->dampening_ = 1.0;
	this->volume_ = 0.0;
	this->density_ = 0.0;
	this->oldVel_ = VECTOR3(0.0, 0.0, 0.0);
	this->oldAngVel_ = VECTOR3(0.0, 0.0, 0.0);
	}

  CompoundBody::CompoundBody(BodyStorage *pBody) : RigidBody(pBody)
  {
    this->affectedByGravity_ = true;
    this->friction_ = 0.0;
    this->dampening_ = 1.0;
    this->volume_ = pBody->volume_;
    this->density_ = pBody->density_;
    this->oldVel_ = VECTOR3(0.0, 0.0, 0.0);
    this->oldAngVel_ = VECTOR3(0.0, 0.0, 0.0);
    shapeId_ = RigidBody::COMPOUND;
    numofComps_ = pBody->spheres;
    shape_ = NULL;
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

	CompoundBody::CompoundBody(const i3d::CompoundBody& copy) : RigidBody(copy)
	{
		//set additional attributes of compounds
		rigidBodies_ = copy.rigidBodies_;
		numofComps_ = copy.numofComps_;
		oldAngVel_ = copy.oldAngVel_;
		oldVel_ = copy.oldVel_;
		
	}


	void CompoundBody::setVolume(){
		/**there are two cases for computing the volume: 
		there either is only an overlap between each 2 spheres of the compound, or 3 or more spheres overlap at the same area.
        the second case is more difficult to compute using standard algebraic methods, and a numerical routine has to be used/
	   2 spheres a, b overlap if radius(a) + radius(b) -|com_(a) - com_(b)| >0
	   */

		//the following computation is only correct for the first case, for the second it provides an incorrect value, since the area where all 
		//3 spheres overlap is substracted 2 more times than needed

		//obtain the three radii of the spheres forming the compound
	  int nradii = this->rigidBodies_.size();
	  std::vector<Real> radii;
    std::vector<Real> volDiff;
	  for(auto &comp : rigidBodies_)
	  {
	    radii.push_back(comp->shape_->getAABB().extents_[0]);
	  }

    for(int i=0;i<rigidBodies_.size();i++)
    {
      for(int j=i;j<rigidBodies_.size();j++)
      {
        Real overlaps = radii[i] + radii[j] - (rigidBodies_[i]->com_ - rigidBodies_[j]->com_).mag();
        if(overlaps > 0.0)
        {
          VECTOR3 dist = rigidBodies_[i]->com_ - rigidBodies_[j]->com_;
          Real d = dist.mag();
          Real v = PI / 12.0 * d*(radii[i] + radii[j] - d)*(radii[i] + radii[j] - d)*(d*d + 2.0 * d*(radii[i] + radii[j]) - 3.0 * (radii[i] - radii[j])*(radii[i] - radii[j]));
          volDiff.push_back(v);
        }
      }
    }

    volume_=0.0;
    Real sumRadii=0.0;
		//the volume of the compound is the volume of the spheres minus the volumes of the overlaps
    for(int i=0;i<rigidBodies_.size();i++)
    {
      sumRadii+=(radii[i]*radii[i]*radii[i]);
    }
    volume_ = (4.0 * PI / 3.0)*(sumRadii);

    for(int i=0;i<volDiff.size();i++)
      volume_ -= volDiff[i];

    Real volTest = (4.0 * PI / 3.0)*(rigidBodies_[0]->shape_->getAABB().extents_[0]*rigidBodies_[0]->shape_->getAABB().extents_[0]*rigidBodies_[0]->shape_->getAABB().extents_[0]);
    Real volTest2 = (4.0 * PI / 3.0)*(rigidBodies_[0]->shape_->getAABB().extents_[0]*rigidBodies_[0]->shape_->getAABB().extents_[0]*rigidBodies_[0]->shape_->getAABB().extents_[0]);

	}

	void CompoundBody::setInvMass(){
		this->invMass_ = 1/(this->volume_* this->density_);
	}
	

	void CompoundBody::translateTo(const VECTOR3& vPos)
	{
		/**to translate all the bodies forming the compound, use this formula:
		   the vector pointing from the old com of the compound com_old to the com of body i com_i is given by com_i-com_old
		   thus, new com of body i com_inew is given by com_new + (com_i - com_old), where com_new represents the new com of the compound*/

    com_ = vPos;
    for (auto &comp : rigidBodies_)
    {
      comp->transform_.setOrigin(com_);
      comp->transform_.setMatrix(getTransformationMatrix());
    }

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
		  RigidBody *body = *i;
		  body->generateInvInertiaTensor();
			InertiaTensor +=body->invInertiaTensor_;
		}
		invInertiaTensor_ = InertiaTensor;
		
	}

	
	void CompoundBody::applyForces(const VECTOR3 &force, const VECTOR3 &torque, const Real &delta)
	{
		MATRIX3X3 mInvInertiaTensor = getWorldTransformedInvTensor();

		//update velocity of the compound
		velocity_ += (delta * force);

		//and the angular velocity
		setAngVel(getAngVel() + mInvInertiaTensor * (delta * torque));

		
		//the velocity of the components changes as well
		std::vector<RigidBody*>::iterator i = rigidBodies_.begin();
		for (; i != rigidBodies_.end(); i++)
		{
      RigidBody* body = *(i);

			//translational velocity of component i is given by 
			// velocity_i =  velocity_ + Cross((com_i -com_), angVel_)
			VECTOR3 crossprod = VECTOR3::Cross( (body->com_ - com_), body->getAngVel() );
			body->velocity_ = velocity_ + crossprod;
			//angVel changed for each component individually
			body->setAngVel(getAngVel());
			
		}
	}

}
